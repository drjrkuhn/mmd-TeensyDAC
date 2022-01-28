///////////////////////////////////////////////////////////////////////////////
// FILE:          TeensyDAC.h
// PROJECT:       Micro-Manager
// SUBSYSTEM:     DeviceAdapters
//-----------------------------------------------------------------------------
// DESCRIPTION:   The example implementation of the demo camera.
//                Simulates generic digital camera and associated automated
//                microscope devices and enables testing of the rest of the
//                system without the need to connect to the actual hardware. 
//                
// AUTHOR:        Nenad Amodaj, nenad@amodaj.com, 06/08/2005
//                
//                Karl Hoover (stuff such as programmable CCD size  & the various image processors)
//                Arther Edelstein ( equipment error simulation)
//
// COPYRIGHT:     University of California, San Francisco, 2006-2015
//                100X Imaging Inc, 2008
//
// LICENSE:       This file is distributed under the BSD license.
//                License text is included with the source distribution.
//
//                This file is distributed in the hope that it will be useful,
//                but WITHOUT ANY WARRANTY; without even the implied warranty
//                of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
//
//                IN NO EVENT SHALL THE COPYRIGHT OWNER OR
//                CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//                INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES.

#ifndef _TEENSYDAC_H_
#define _TEENSYDAC_H_

#define LOG_DEVICE_HEX_PROTOCOL

#include "DeviceBase.h"
#include "ImgBuffer.h"
#include "DeviceThreads.h"
#include <string>
#include <map>
#include <algorithm>
#include <cstdint>

#include "HexProtocol.h"
#include "DeviceHexProtocol.h"
#include "TeensyDACProtocol.h"
#include "DeviceProp.h"
#include "LocalProp.h"
#include "RemoteProp.h"
#include "DeviceCommon.h"

/** \ingroup SShedShutter
Serial Device "Verbose" property. Depends on 
whether LOG_DEVICE_HEX_PROTOCOL is #defined */
#ifdef LOG_DEVICE_HEX_PROTOCOL
#define SERIAL_VERBOSE	"1"
#else
#define SERIAL_VERBOSE	"0"
#endif

namespace tdac {

	using namespace dprop;

	//#############################################################################
	//### Constants
	//#############################################################################

	////////////////////////////////////////////////////////////////
	/// \name Module-level constants
	/// \ingroup TeensyDAC
	///@{
	const char* g_deviceNameHub = "TeensyDAC-Hub";
	const char* g_deviceDescHub = "KI TeensyDAC Hub";
	const char* g_deviceNameGalvo = "TeensyDAC-Gal";
	const char* g_deviceDescGalvo = "Galvo Axis";
#if defined(HAS_SHUTTERS)
	//const char* g_deviceNameShutter = "TeensyDAC-sh";
	//const char* g_deviceDescShutter = "Shutter";
#endif
	//@}

	////////////////////////////////////////////////////////////////
	/// \name PropInfo local property constants
	/// \ingroup TeensyDAC
	///@{
	const auto g_infoPort = PropInfo<std::string>::build(MM::g_Keyword_Port, "Undefined").withIsPreInit();
	///@}

	////////////////////////////////////////////////////////////////
	/// \name PropInfo remote device property constants
	/// \ingroup TeensyDAC
	///@{
	const auto g_infoFirmware = PropInfo<::std::string>::build("Firmware Name", "Undefined").assertReadOnly();
	const auto g_infoVersion = PropInfo<version_t>::build("Firmware Version", 0).assertReadOnly();
	const auto g_infoRevision = PropInfo<version_t>::build("Firmware Revision", 0).assertReadOnly();
	const auto g_infoIsAlive = PropInfo<::hprot::prot_bool_t>::build("Is Alive", false).assertReadOnly();
	const auto g_infoNumGalvos = PropInfo<channel_t>::build("Number of Galvos", MAX_NUM_GALVOS);
#if defined(HAS_SHUTTERS)
	const auto g_infoNumShutters = PropInfo<channel_t>::build("Number of Shutters", MAX_NUM_SHUTTERS);
#endif

	///@}

	////////////////////////////////////////////////////////////////
	/// \name PropInfo remote state property constants
	/// \ingroup TeensyDAC
	///@{
	const auto g_infoGalvoOffset = PropInfo<position_t>::build("OffV", 0).withLimits(-10.0, 10.0);
	const auto g_infoGalvoPosition = PropInfo<position_t>::build("PosV", 0).withLimits(-10.0, 10.0);
	const auto g_infoGalvoFrequency = PropInfo<frequency_t>::build("FreqHz", 0).withLimits(0.0, 10000.0);
#if defined(GALVOS_SEQUENCABLE)
	const auto g_infoGalvoPosSeq = PropInfo<std::string>::build("Pos Sequence", "-").assertReadOnly();
#endif
#if defined(HAS_SHUTTERS)
	const auto g_infoShutterOpen = PropInfo<::hprot::prot_bool_t>::build("Open", 0).withAllowedValues({ 0, 1 });
#endif
	///@}


	//#############################################################################
	//### class TeensyDACHub
	//#############################################################################

	/** \ingroup TeensyDAC
		Brimrose arduino control device implementation.
		*/
	class TeensyDACHub : public HubBase<TeensyDACHub>, public hprot::DeviceHexProtocol<TeensyDACHub> {
	public:
		typedef TeensyDACHub CLASS;
		typedef TeensyDACHub HUB;
		typedef hprot::DeviceHexProtocol<CLASS> PROTOCOL;

		////////////////////////////////////////////////////////////////
		/// \name TeensyDACHub constructors and destructors
		////////////////////////////////////////////////////////////////
		///@{
		TeensyDACHub();
		virtual ~TeensyDACHub();
		///@}

		//////////////////////////////////////////////////////////
		/// \name MM::Device implementation
		//////////////////////////////////////////////////////////
		///@{
		void GetName(char* __name) const override {
			CDeviceUtils::CopyLimitedString(__name, g_deviceNameHub);
		}
		bool Busy() override;
		int Shutdown() override;
		int Initialize() override;
		MM::DeviceDetectionStatus DetectDevice(void) override;
		///@}


		//////////////////////////////////////////////////////////
		/// \name MM::HubBase implementation
		//////////////////////////////////////////////////////////
		///@{
		int DetectInstalledDevices() override;
		///@}


		////////////////////////////////////////////////////////////////
		/// \name DeviceHexProtocol implementation
		////////////////////////////////////////////////////////////////
		int testProtocol(void) override;
		///@}

		////////////////////////////////////////////////////////////////
		/// \name Custom interface for child devices
		////////////////////////////////////////////////////////////////
		bool isPortAvailable() { return portAvailable_; }
		///@}

	protected:
		friend class PROTOCOL;
		bool initialized_;
		bool portAvailable_;

		////////////////////////////////////////////////////////////////
		/// \name local properties
		////////////////////////////////////////////////////////////////
		///@{
		LocalProp<::std::string, CLASS> port_;
		///@}

		////////////////////////////////////////////////////////////////
		/// \name remote properties
		////////////////////////////////////////////////////////////////
		///@{
		RemoteReadOnlyProp<::std::string, CLASS, HUB> firmwareName_;
		RemoteReadOnlyProp<version_t, CLASS, HUB> firmwareVersion_;
		RemoteReadOnlyProp<version_t, CLASS, HUB> firmwareRevision_;
		RemoteReadOnlyProp<::hprot::prot_bool_t, CLASS, HUB> isAlive_;
		RemoteReadOnlyProp<channel_t, CLASS, HUB> numGalvos_;
#if defined(HAS_SHUTTERS)
		RemoteReadOnlyProp<channel_t, CLASS, HUB> numShutters_;
#endif
		///@}

	}; // class TeensyDACHub


	//#############################################################################
	//### class TeensyDACGalvo
	//#############################################################################

	/** \ingroup TeensyDAC
		Amplitude Galvo Position Device
		*/
	class TeensyDACGalvo : public CStageBase<TeensyDACGalvo> {
		typedef TeensyDACGalvo CLASS;
		typedef TeensyDACHub HUB;
	public:
		////////////////////////////////////////////////////////////////
		/// \name constructors and destructors
		////////////////////////////////////////////////////////////////
		///@{
		TeensyDACGalvo(channel_t	__chan);
		virtual ~TeensyDACGalvo();
		///@}

		//////////////////////////////////////////////////////////
		/// \name MM::Device implementation
		//////////////////////////////////////////////////////////
		///@{
		void GetName(char* __name) const override {
			::std::string name = g_deviceNameGalvo;
			name += char('A' + chan_);
			CDeviceUtils::CopyLimitedString(__name, name.c_str());
		}
		bool Busy() override;
		int Shutdown() override;
		int Initialize() override;
		///@}

		//////////////////////////////////////////////////////////
		/// \name MM::Stage implementation
		///
		/// NOTE: Although Stage Position function is in um,
		///       we will use galvo position in terms of voltage
		///		  as this fits better with MM's MD acquisition
		//////////////////////////////////////////////////////////
		///@{
		int SetPositionUm(double pos) override;
		int GetPositionUm(double& pos) override;
		double GetStepSize() const;
		int SetPositionSteps(long steps) override;
		int GetPositionSteps(long& steps) override;
		int GetLimits(double& lower, double& upper) override;

		int SetOrigin() override { return DEVICE_OK; }
		int Move(double /*v*/) override { return DEVICE_OK; }
		bool IsContinuousFocusDrive() const override { return false; }

		// Sequence functions
		int IsStageSequenceable(bool&) const override { 
#if defined(GALVOS_SEQUENCABLE)
			return true;
#else
			return false;
#endif
		};
#if defined(GALVOS_SEQUENCABLE)
		int GetStageSequenceMaxLength(long& nrEvents) const override;
		int StartStageSequence() override;
		int StopStageSequence() override;
		int ClearStageSequence() override;
		int AddToStageSequence(double pos_um) override;
		int SendStageSequence() override;
#endif
		///@}

	protected:
		channel_t	chan_;
#if defined(GALVOS_SEQUENCABLE)
		RemoteSequenceableProp<position_t, CLASS, HUB> pos_;
		RemoteArrayProp<position_t, CLASS, HUB> posSeqRead_;
		std::vector<std::string> posSequence_;
#else
		RemoteProp<position_t, CLASS, HUB> pos_;
#endif
		RemoteProp<position_t, CLASS, HUB> off_;
		RemoteProp<frequency_t, CLASS, HUB> freq_;


	}; // TeensyDACGalvo


#if defined(HAS_SHUTTERS)
	//#############################################################################
	//### class TeensyDACShutter
	//#############################################################################

	/** \ingroup TeensyDAC
		Amplitude Shutter Device
		*/
	class TeensyDACShutter : public CShutterBase<TeensyDACShutter> {
		typedef TeensyDACShutter CLASS;
		typedef TeensyDACHub HUB;
	public:
		////////////////////////////////////////////////////////////////
		/// \name constructors and destructors
		////////////////////////////////////////////////////////////////
		///@{
		TeensyDACShutter(channel_t	__chan);
		virtual ~TeensyDACShutter();
		///@}

		//////////////////////////////////////////////////////////
		/// \name MM::Device implementation
		//////////////////////////////////////////////////////////
		///@{
		void GetName(char* __name) const override {
			::std::string name = g_deviceNameShutter;
			name += char('A' + chan_);
			CDeviceUtils::CopyLimitedString(__name, name.c_str());
		}
		bool Busy() override;
		int Shutdown() override;
		int Initialize() override;
		///@}

		//////////////////////////////////////////////////////////
		/// \name MM::Shutter implementation
		//////////////////////////////////////////////////////////
		///@{
		int SetOpen(bool __open = true) override;
		int GetOpen(bool& __open) override;
		int Fire(double /*__deltaT*/) override {
			return DEVICE_UNSUPPORTED_COMMAND;
		}

	protected:
		channel_t	chan_;
		RemoteProp<::hprot::prot_bool_t, CLASS, HUB> open_;
	}; // TeensyDACShutter
#endif // HAS_SHUTTERS

}; // namespace tdac

#endif //_TEENSYDAC_H_
