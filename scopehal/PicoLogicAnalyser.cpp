/***********************************************************************************************************************
*                                                                                                                      *
* libscopehal v0.1                                                                                                     *
*                                                                                                                      *
* Copyright (c) 2012-2023 Andrew D. Zonenberg and contributors                                                         *
* All rights reserved.                                                                                                 *
*                                                                                                                      *
* Redistribution and use in source and binary forms, with or without modification, are permitted provided that the     *
* following conditions are met:                                                                                        *
*                                                                                                                      *
*    * Redistributions of source code must retain the above copyright notice, this list of conditions, and the         *
*      following disclaimer.                                                                                           *
*                                                                                                                      *
*    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the       *
*      following disclaimer in the documentation and/or other materials provided with the distribution.                *
*                                                                                                                      *
*    * Neither the name of the author nor the names of any contributors may be used to endorse or promote products     *
*      derived from this software without specific prior written permission.                                           *
*                                                                                                                      *
* THIS SOFTWARE IS PROVIDED BY THE AUTHORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED   *
* TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL *
* THE AUTHORS BE HELD LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES        *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR       *
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE       *
* POSSIBILITY OF SUCH DAMAGE.                                                                                          *
*                                                                                                                      *
***********************************************************************************************************************/

#ifdef _WIN32
#include <chrono>
#include <thread>
#endif

#include "scopehal.h"
#include "PicoLogicAnalyser.h"
#include "EdgeTrigger.h"

using namespace std;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Construction / destruction

PicoLogicAnalyser::PicoLogicAnalyser(SCPITransport* transport)
	: SCPIDevice(transport), SCPIInstrument(transport), RemoteBridgeOscilloscope(transport), m_digitalChannelCount(8)
{
	IdentifyHardware();

	// Add digital channels (named 1D0...7 and 2D0...7)
	for(size_t i = 0; i < m_digitalChannelCount; i++)
	{
		size_t ichan = i;
		string chname = "D";
		chname += std::to_string(i);

		// Create the channel
		auto chan = new OscilloscopeChannel(this,
			chname,
			GetChannelColor(ichan),
			Unit(Unit::UNIT_FS),
			Unit(Unit::UNIT_COUNTS),
			Stream::STREAM_TYPE_DIGITAL,
			i);
		m_channels.push_back(chan);
		m_channelsEnabled[i] = true;
		chan->SetDefaultDisplayName();
	}

	SetSampleRate(GetSampleRatesNonInterleaved().at(0));
	SetSampleDepth(GetSampleDepthsNonInterleaved().at(0));
	SetNumChannels();

	//Configure the trigger
	auto trig = new EdgeTrigger(this);
	trig->SetType(EdgeTrigger::EDGE_RISING);
	trig->SetLevel(0);
	trig->SetInput(0, StreamDescriptor(GetOscilloscopeChannel(0)));
	SetTrigger(trig);
	PushTrigger();
	SetTriggerOffset(0);

	//Create Vulkan objects for the waveform conversion
	m_queue = g_vkQueueManager->GetComputeQueue("PicoLogicAnalyser.queue");
	vk::CommandPoolCreateInfo poolInfo(
		vk::CommandPoolCreateFlagBits::eTransient | vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
		m_queue->m_family);
	m_pool = make_unique<vk::raii::CommandPool>(*g_vkComputeDevice, poolInfo);

	vk::CommandBufferAllocateInfo bufinfo(**m_pool, vk::CommandBufferLevel::ePrimary, 1);
	m_cmdBuf =
		make_unique<vk::raii::CommandBuffer>(std::move(vk::raii::CommandBuffers(*g_vkComputeDevice, bufinfo).front()));

	if(g_hasDebugUtils)
	{
		string poolname = "PicoLogicAnalyser.pool";
		string bufname = "PicoLogicAnalyser.cmdbuf";

		g_vkComputeDevice->setDebugUtilsObjectNameEXT(vk::DebugUtilsObjectNameInfoEXT(vk::ObjectType::eCommandPool,
			reinterpret_cast<uint64_t>(static_cast<VkCommandPool>(**m_pool)),
			poolname.c_str()));

		g_vkComputeDevice->setDebugUtilsObjectNameEXT(vk::DebugUtilsObjectNameInfoEXT(vk::ObjectType::eCommandBuffer,
			reinterpret_cast<uint64_t>(static_cast<VkCommandBuffer>(**m_cmdBuf)),
			bufname.c_str()));
	}

	m_conversionPipeline =
		make_unique<ComputePipeline>("shaders/Convert16BitSamples.spv", 2, sizeof(ConvertRawSamplesShaderArgs));
}

/**
	@brief Color the channels based on Pico's standard color sequence (blue-red-green-yellow-purple-gray-cyan-magenta)
 */
string PicoLogicAnalyser::GetChannelColor(size_t i)
{
	switch(i % 8)
	{
		case 0:
			return "#4040ff";

		case 1:
			return "#ff4040";

		case 2:
			return "#208020";

		case 3:
			return "#ffff00";

		case 4:
			return "#600080";

		case 5:
			return "#808080";

		case 6:
			return "#40a0a0";

		case 7:
		default:
			return "#e040e0";
	}
}

void PicoLogicAnalyser::IdentifyHardware()
{
	LogWarning("PicoScope model \"%s\"\n", m_model.c_str());
}

PicoLogicAnalyser::~PicoLogicAnalyser()
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Accessors

unsigned int PicoLogicAnalyser::GetInstrumentTypes() const
{
	return Instrument::INST_OSCILLOSCOPE;
}

uint32_t PicoLogicAnalyser::GetInstrumentTypesForChannel(size_t) const
{
	return Instrument::INST_OSCILLOSCOPE;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Device interface functions

string PicoLogicAnalyser::GetDriverNameInternal()
{
	return "picola";
}

void PicoLogicAnalyser::FlushConfigCache()
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);

	//clear probe presence flags as those can change without our knowledge
	m_digitalBankPresent.clear();
}

bool PicoLogicAnalyser::IsChannelEnabled(size_t i)
{
	return m_channelsEnabled[i];
}

void PicoLogicAnalyser::SetNumChannels()
{
	size_t last_enabled_channel = 0;
	for(size_t i = 0; i < m_digitalChannelCount; i++)
	{
		if(m_channelsEnabled[i])
		{
			last_enabled_channel = i;
		}
	}

	std::string command = "CHANS " + std::to_string(last_enabled_channel + 1);
	m_transport->SendCommand(command);
	SetSampleDepth(GetSampleDepthsNonInterleaved().at(0));
}

size_t PicoLogicAnalyser::DigitalChannelsActive()
{
	size_t channels = 0;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->FlushRXBuffer();
		m_transport->SendCommand("CHANS?");
		channels = (size_t)stoi(m_transport->ReadReply());
	}

	return channels;
}

void PicoLogicAnalyser::EnableChannel(size_t i)
{
	m_channelsEnabled[i] = true;
	SetNumChannels();
}

void PicoLogicAnalyser::DisableChannel(size_t i)
{
	m_channelsEnabled[i] = false;
	SetNumChannels();
}

vector<OscilloscopeChannel::CouplingType> PicoLogicAnalyser::GetAvailableCouplings(size_t /*i*/)
{
	vector<OscilloscopeChannel::CouplingType> ret;
	ret.push_back(OscilloscopeChannel::COUPLE_DC_1M);
	return ret;
}

double PicoLogicAnalyser::GetChannelAttenuation(size_t i)
{
	if(GetOscilloscopeChannel(i) == m_extTrigChannel)
		return 1;

	lock_guard<recursive_mutex> lock(m_cacheMutex);
	return m_channelAttenuations[i];
}

void PicoLogicAnalyser::SetChannelAttenuation(size_t i, double atten)
{
	lock_guard<recursive_mutex> lock(m_cacheMutex);
	double oldAtten = m_channelAttenuations[i];
	m_channelAttenuations[i] = atten;

	// Rescale channel voltage range and offset
	double delta = atten / oldAtten;
	m_channelVoltageRanges[i] *= delta;
	m_channelOffsets[i] *= delta;
}

unsigned int PicoLogicAnalyser::GetChannelBandwidthLimit(size_t /*i*/)
{
	return 0;
}

void PicoLogicAnalyser::SetChannelBandwidthLimit(size_t /*i*/, unsigned int /*limit_mhz*/)
{
}

OscilloscopeChannel* PicoLogicAnalyser::GetExternalTrigger()
{
	//FIXME
	return NULL;
}

Oscilloscope::TriggerMode PicoLogicAnalyser::PollTrigger()
{
	//Always report "triggered" so we can block on AcquireData() in ScopeThread
	//TODO: peek function of some sort?
	return TRIGGER_MODE_TRIGGERED;
}

// TODO(josh): lots of assumptions here, assuming members reflect state of LA at time of aquisition. This isn't necessarily true,
// this information could be added to the data in the data aquisition
bool PicoLogicAnalyser::AcquireData()
{
	const size_t word_size_bytes = 4;
	const size_t digital_channels_active = DigitalChannelsActive();
	LogWarning("DigitalChannelsActive %lu", digital_channels_active);
	const size_t samples_per_word = (word_size_bytes * 8) - ((word_size_bytes * 8) % digital_channels_active);
	const size_t words_per_capture = (GetSampleDepth() * digital_channels_active) / samples_per_word;
	uint32_t* buf = new uint32_t[words_per_capture];

	// Read data
	if(!m_transport->ReadRawData(words_per_capture * word_size_bytes, (unsigned char*)buf))
	{
		LogWarning("Couldn't read data from socket\n");
		delete[] buf;
		return false;
	}

	// Create waveform for each channel
	SparseDigitalWaveform* caps[16];	// Max possible channels
	SequenceSet s;
	for(size_t channel_idx = 0; channel_idx < digital_channels_active; channel_idx++)
	{
		caps[channel_idx] =
			AllocateDigitalWaveform(m_nickname + "." + GetOscilloscopeChannel(channel_idx)->GetHwname());
		s[GetOscilloscopeChannel(channel_idx)] = caps[channel_idx];
	}

	// Unpack waveform
	const double now = GetTime();
	for(size_t channel_idx = 0; channel_idx < digital_channels_active; channel_idx++)
	{
		auto cap = caps[channel_idx];
		cap->m_timescale = FS_PER_SECOND / GetSampleRate();
		cap->m_triggerPhase = 0;
		cap->m_startTimestamp = floor(now);
		cap->m_startFemtoseconds = (now - floor(now)) * FS_PER_SECOND;

		// Preallocate memory assuming no deduplication possible
		cap->Resize(m_mdepth);
		cap->PrepareForCpuAccess();

		for(size_t sample_idx = 0; sample_idx < m_mdepth; sample_idx++)
		{
			const size_t bit_index = (channel_idx + sample_idx * digital_channels_active);
			const size_t buf_index = bit_index / 32;
			const size_t word_index = bit_index % 32;
			const bool sample = (buf[buf_index] >> word_index) & 1;

			cap->m_offsets[sample_idx] = sample_idx;
			cap->m_durations[sample_idx] = 1;
			cap->m_samples[sample_idx] = sample;
		}
		cap->MarkSamplesModifiedFromCpu();
		cap->MarkTimestampsModifiedFromCpu();
	}
	//Just in case we missed anything from the capture
	m_transport->FlushRXBuffer();

	delete[] buf;

	//Save the waveforms to our queue
	m_pendingWaveformsMutex.lock();
	m_pendingWaveforms.push_back(s);
	m_pendingWaveformsMutex.unlock();

	//If this was a one-shot trigger we're no longer armed
	if(m_triggerOneShot)
	{
		m_triggerArmed = false;
	}

	ChannelsDownloadFinished();

	return true;
}

bool PicoLogicAnalyser::IsTriggerArmed()
{
	return m_triggerArmed;
}

bool PicoLogicAnalyser::CanInterleave()
{
	return false;
}

vector<uint64_t> PicoLogicAnalyser::GetSampleRatesNonInterleaved()
{
	vector<uint64_t> ret;

	string rates;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->FlushRXBuffer();
		m_transport->SendCommand("RATES?");
		rates = m_transport->ReadReply();
	}

	size_t i = 0;
	while(true)
	{
		size_t istart = i;
		i = rates.find(',', i + 1);
		if(i == string::npos)
			break;

		auto block = rates.substr(istart, i - istart);
		uint64_t hz = stoull(block);
		ret.push_back(hz);

		//skip the comma
		i++;
	}

	return ret;
}

vector<uint64_t> PicoLogicAnalyser::GetSampleRatesInterleaved()
{
	//interleaving not supported
	vector<uint64_t> ret = {};
	return ret;
}

set<Oscilloscope::InterleaveConflict> PicoLogicAnalyser::GetInterleaveConflicts()
{
	//interleaving not supported
	set<Oscilloscope::InterleaveConflict> ret;
	return ret;
}

vector<uint64_t> PicoLogicAnalyser::GetSampleDepthsNonInterleaved()
{
	vector<uint64_t> ret;

	string depths;
	{
		lock_guard<recursive_mutex> lock(m_mutex);
		m_transport->FlushRXBuffer();
		m_transport->SendCommand("DEPTHS?");
		depths = m_transport->ReadReply();
	}

	size_t i = 0;
	while(true)
	{
		size_t istart = i;
		i = depths.find(',', i + 1);
		if(i == string::npos)
			break;

		uint64_t sampleDepth = stoull(depths.substr(istart, i - istart));
		ret.push_back(sampleDepth);

		//skip the comma
		i++;
	}

	return ret;
}

vector<uint64_t> PicoLogicAnalyser::GetSampleDepthsInterleaved()
{
	//interleaving not supported
	vector<uint64_t> ret;
	return ret;
}

uint64_t PicoLogicAnalyser::GetSampleRate()
{
	return m_srate;
}

uint64_t PicoLogicAnalyser::GetSampleDepth()
{
	return m_mdepth;
}

void PicoLogicAnalyser::SetSampleDepth(uint64_t depth)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(string("DEPTH ") + to_string(depth));
	m_mdepth = depth;
}

void PicoLogicAnalyser::SetSampleRate(uint64_t rate)
{
	lock_guard<recursive_mutex> lock(m_mutex);
	m_transport->SendCommand(string("RATE ") + to_string(rate));
	m_srate = rate;
}

void PicoLogicAnalyser::SetTriggerOffset(int64_t offset)
{
	lock_guard<recursive_mutex> lock(m_mutex);

	//Don't allow setting trigger offset beyond the end of the capture
	int64_t captureDuration = GetSampleDepth() * FS_PER_SECOND / GetSampleRate();
	m_triggerOffset = min(offset, captureDuration);

	PushTrigger();
}

int64_t PicoLogicAnalyser::GetTriggerOffset()
{
	return m_triggerOffset;
}

bool PicoLogicAnalyser::IsInterleaving()
{
	//interleaving is done automatically in hardware based on sample rate, no user facing switch for it
	return false;
}

bool PicoLogicAnalyser::SetInterleaving(bool /*combine*/)
{
	//interleaving is done automatically in hardware based on sample rate, no user facing switch for it
	return false;
}

void PicoLogicAnalyser::PushTrigger()
{
	auto et = dynamic_cast<EdgeTrigger*>(m_trigger);
	if(et)
		PushEdgeTrigger(et);

	else
		LogWarning("Unknown trigger type (not an edge)\n");

	ClearPendingWaveforms();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Logic analyzer configuration

vector<Oscilloscope::DigitalBank> PicoLogicAnalyser::GetDigitalBanks()
{
	vector<DigitalBank> banks;
	for(size_t i = 0; i < m_digitalChannelCount; i++)
	{
		DigitalBank bank;
		bank.push_back(GetOscilloscopeChannel(i));
		banks.push_back(bank);
	}
	return banks;
}

Oscilloscope::DigitalBank PicoLogicAnalyser::GetDigitalBank(size_t channel)
{
	DigitalBank ret;
	ret.push_back(GetOscilloscopeChannel(channel));
	return ret;
}
