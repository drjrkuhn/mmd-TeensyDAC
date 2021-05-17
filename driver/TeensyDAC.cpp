///////////////////////////////////////////////////////////////////////////////
// FILE:          TeensyDAC.cpp
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
// COPYRIGHT:     University of California, San Francisco, 2006
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

#include "MMDevice.h"
#include "DeviceBase.h"
#include "ModuleInterface.h"
#include "TeensyDAC.h"


/** Time to wait for the remote (arduino) to reset after the initial DTR connection */
#define REMOTE_INIT_DELAY_MS	2000

using namespace dprop;

using namespace tdac;

//#############################################################################
//### Module
//#############################################################################

///////////////////////////////////////////////////////////////////////////////
/// \name Exported Module API
/// @see ModuleInterface.h
///////////////////////////////////////////////////////////////////////////////
///@{

/** Initialize the device adapter module. */
MODULE_API void InitializeModuleData() {
	RegisterDevice(g_deviceNameHub, MM::HubDevice, g_deviceDescHub);
	for (int i = 0; i < MAX_NUM_GALVOS; i++) {
		std::string name = g_deviceNameGalvo;
		std::string desc = g_deviceDescGalvo;
		name += char('A' + i);
		desc += " ";
		desc += char('A' + i);
		RegisterDevice(name.c_str(), MM::StageDevice, desc.c_str());
	}
#if defined(HAS_SHUTTERS)
	for (int i = 0; i < MAX_NUM_SHUTTERS; i++) {
		std::string name = g_deviceNameShutter;
		std::string desc = g_deviceDescShutter;
		name += char('A' + i);
		desc += " ";
		desc += char('A' + i);
		RegisterDevice(name.c_str(), MM::ShutterDevice, desc.c_str());
	}
#endif
}

/** Instantiate the named device. */
MODULE_API MM::Device* CreateDevice(const char* deviceName) {
	if (deviceName == 0) {
		return nullptr;
	}
	if (strcmp(deviceName, g_deviceNameHub) == 0) {
		return new TeensyDACHub;
	}

	for (channel_t i = 0; i < MAX_NUM_GALVOS; i++) {
		std::string name = g_deviceNameGalvo;
		name += char('A' + i);
		if (strcmp(deviceName, name.c_str()) == 0) {
			return new TeensyDACGalvo(i);
		}
	}
#if defined(HAS_SHUTTERS)
	for (channel_t i = 0; i < MAX_NUM_SHUTTERS; i++) {
		std::string name = g_deviceNameShutter;
		name += char('A' + i);
		if (strcmp(deviceName, name.c_str()) == 0) {
			return new TeensyDACShutter(i);
		}
	}
#endif
	return nullptr;
}

/** Destroy a device instance. */
MODULE_API void DeleteDevice(MM::Device* pDevice) {
	delete pDevice;
}

///@}

//#############################################################################
//### class TeensyDACHub
//#############################################################################

///////////////////////////////////////////////////////////////////////////////
/// \name TeensyDACHub constructors and destructors
///////////////////////////////////////////////////////////////////////////////
///@{

/** Default constructors. Sets creates the Pre-Init serial port string property. */
TeensyDACHub::TeensyDACHub() : initialized_(false), portAvailable_(false) {

	InitializeDefaultErrorMessages();
	initCommonErrors("Teensy", CURRENT_VERSION, [this](int err, const char* txt) {
		SetErrorText(err, txt);
	});
	port_.createLocalProp(this, tdac::g_infoPort);
}

/** Default destructor. Calls Shutdown(). */
TeensyDACHub::~TeensyDACHub() {
	TeensyDACHub::Shutdown();
}

///@}

///////////////////////////////////////////////////////////////////////////////
/// \name TeensyDACHub MM::Device implementation
///////////////////////////////////////////////////////////////////////////////
///@{

/** No device threads and all commands block. So the hub is never "Busy". */
bool TeensyDACHub::Busy() {
	return false;
}

/** Device discovery (auto-configuration).
Determine if a given serial port is actively connected to a valid
slave device. Serial port settings are handled by tryStream. */
MM::DeviceDetectionStatus TeensyDACHub::DetectDevice(void) {
	if (initialized_)
		return MM::CanCommunicate;
	return tryStream(this, port_.getCachedValue(), BAUDRATE);
}


/** Add all of the slave devices */
int TeensyDACHub::DetectInstalledDevices()
{
	if (MM::CanCommunicate == DetectDevice()) {
		channel_t numGalvos = 0;
		if (!dispatchGet(GET_NUM_GALVOS, numGalvos)) {
			return ERR_COMMUNICATION;
		}
#if defined(HAS_SHUTTERS)
		channel_t numShutters = 0;
		if (!dispatchGet(GET_NUM_SHUTTERS, numShutters)) {
			return ERR_COMMUNICATION;
		}
#endif
		for (int i = 0; i < numGalvos; i++) {
			std::string name = g_deviceNameGalvo;
			name += char('A' + i);
			MM::Device* pDev = ::CreateDevice(name.c_str());
			if (pDev) {
				AddInstalledDevice(pDev);
			}
		}
#if defined(HAS_SHUTTERS)
		for (int i = 0; i < numShutters; i++) {
			std::string name = g_deviceNameShutter;
			name += char('A' + i);
			MM::Device* pDev = ::CreateDevice(name.c_str());
			if (pDev) {
				AddInstalledDevice(pDev);
			}
		}
#endif
	}

	return DEVICE_OK;
}

///@}

///////////////////////////////////////////////////////////////////////////////
/// \name TeensyDACHub DeviceHexProtocol implementation
///////////////////////////////////////////////////////////////////////////////
///@{

int TeensyDACHub::testProtocol(void) {
	version_t version = 0, revision = 0;
	std::string firmware;
	portAvailable_ = false;

	// Get a series of values, stopping at the first problem
	if (!dispatchGet(GET_FIRMWARE, firmware)) {
		return ERR_COMMUNICATION;
	}

	if (!dispatchGet(GET_VERSION, version)) {
		return ERR_COMMUNICATION;
	}

	if (!dispatchGet(GET_REVISION, revision)) {
		return ERR_COMMUNICATION;
	}

	// Check the firmware string
	if (firmware != FIRMWARE_STR) {
		return ERR_FIRMWARE_NOT_FOUND;
	}

	// We will check the version and revision numbers during Initialize()
	portAvailable_ = true;
	return DEVICE_OK;
}

///@}

///////////////////////////////////////////////////////////////////////////////
/// \name TeensyDACHub Shutdown and Initialization
///////////////////////////////////////////////////////////////////////////////
///@{

/** Shutdown the remote device. Sends a shutdown task to the remote. */
int TeensyDACHub::Shutdown() {
	if (initialized_) {
		if (!dispatchTask(TASK_SHUTDOWN)) {
			LogMessage("Could not shut-down the remote device");
			return DEVICE_ERR;
		}
	}
	endProtocol();
	initialized_ = false;
	portAvailable_ = false;
	return DEVICE_OK;
}

/** Initialize the device and all of its properties. */
int TeensyDACHub::Initialize() {
	try {
		// Simple name property
		assertOK(CreateProperty(MM::g_Keyword_Name, g_deviceNameHub, MM::String, true));

		// get a copy of the port string, start the protocol, and make sure the device is alive
		std::string port = port_.getCachedValue();
		startProtocol(this, port);
		// Set serial device's Verbose property depending on the LOG_DEVICE_HEX_PROTOCOL flag
		assertOK(GetCoreCallback()->SetDeviceProperty(port.c_str(), "Verbose", SERIAL_VERBOSE));
		// The first second or so after opening the serial port, the Arduino is 
		// waiting for firmwareupgrades.  Simply sleep 2 seconds.
		CDeviceUtils::SleepMs(REMOTE_INIT_DELAY_MS);
		assertOK(purgeComPort());
		assertOK(testProtocol());

		// Create the remote firmware, version, and revision properties. 
		// Because they are read-only, they are read immediately.

		CommandSet cmds = CommandSet::build().withGet(GET_FIRMWARE);
		assertOK(firmwareName_.createRemoteProp(this, this, g_infoFirmware, cmds));

		cmds = CommandSet::build().withGet(GET_VERSION);
		assertOK(firmwareVersion_.createRemoteProp(this, this, g_infoVersion, cmds));

		cmds = CommandSet::build().withGet(GET_REVISION);
		assertOK(firmwareRevision_.createRemoteProp(this, this, g_infoRevision, cmds));

		// Check the version and revision
		if (firmwareVersion_.getCachedValue() < CURRENT_VERSION || firmwareRevision_.getCachedValue() > CURRENT_REVISION) {
			throw ERR_VERSION_MISMATCH;
		}

		cmds = CommandSet::build().withGet(GET_IS_ALIVE);
		assertOK(isAlive_.createRemoteProp(this, this, g_infoIsAlive, cmds));

		cmds = CommandSet::build().withGet(GET_NUM_GALVOS);
		assertOK(numGalvos_.createRemoteProp(this, this, g_infoNumGalvos, cmds));

#if defined(HAS_SHUTTERS)
		cmds = CommandSet::build().withGet(GET_NUM_SHUTTERS);
		assertOK(numShutters_.createRemoteProp(this, this, g_infoNumShutters, cmds));
#endif

	} catch (DeviceResultException deviceError) {
		LogMessage(deviceError.format(this));
		std::string lastTrans = getLastLog();
		if (!lastTrans.empty()) {
			LogMessage(std::string("Last Transaction: ") + lastTrans);
		}
		return deviceError.error;
	}
	initialized_ = true;
	return DEVICE_OK;
}

///@}




//#############################################################################
//### class TeensyDACGalvo
//#############################################################################


////////////////////////////////////////////////////////////////
/// \name constructors and destructors
////////////////////////////////////////////////////////////////
///@{
TeensyDACGalvo::TeensyDACGalvo(channel_t __chan) : chan_(__chan) {
	InitializeDefaultErrorMessages();
	initCommonErrors("Arduino", CURRENT_VERSION, [this](int err, const char* txt) {
		SetErrorText(err, txt);
	});
#if defined(GALVOS_SEQUENCABLE)
	posSequence_.clear();
#endif
}

TeensyDACGalvo::~TeensyDACGalvo() {
	TeensyDACGalvo::Shutdown();
}
///@}

//////////////////////////////////////////////////////////
/// \name MM::Device implementation
//////////////////////////////////////////////////////////
///@{
int TeensyDACGalvo::Shutdown() {
	return DEVICE_OK;
}

int TeensyDACGalvo::Initialize() {
	TeensyDACHub* hub = static_cast<TeensyDACHub*>(GetParentHub());
	if (!hub || !hub->isPortAvailable()) {
		return ERR_NO_PORT_SET;
	}
	try {
		CommandSet cmds = CommandSet::build();	// dummy to initialize

#if defined(GALVOS_SEQUENCABLE)
		cmds = CommandSet::build().withChan(chan_).withSet(GAL_SET_POS).withGet(GAL_GET_POS)
			.withSetSeq(GAL_SETSEQ_POS).withGetSeq(GAL_GETSEQ_POS)
			.withStartSeq(GAL_STARTSEQ_POS).withStopSeq(GAL_STOPSEQ_POS);
		assertOK(pos_.createRemoteProp(this, hub, g_infoGalvoPosition, cmds));
		cmds = CommandSet::build().withGet(GAL_GET_ARRAYSEQ_POS);
		assertOK(posSeqRead_.createRemoteProp(this, hub, g_infoGalvoPosSeq, cmds));

#else
		cmds = CommandSet::build().withChan(chan_).withSet(GAL_SET_POS).withGet(GAL_GET_POS);
		assertOK(pos_.createRemoteProp(this, hub, g_infoGalvoPosition, cmds));
#endif

		cmds = CommandSet::build().withChan(chan_).withSet(GAL_SET_FREQ).withGet(GAL_GET_FREQ);
		assertOK(freq_.createRemoteProp(this, hub, g_infoGalvoFrequency, cmds));

	} catch (DeviceResultException deviceError) {
		LogMessage(deviceError.format(this));
		std::string lastTrans = hub->getLastLog();
		if (!lastTrans.empty()) {
			LogMessage(std::string("Last Transaction: ") + lastTrans);
		}
		return deviceError.error;
	}
	return DEVICE_OK;
}
///@}

//////////////////////////////////////////////////////////
/// \name MM::Device implementation
//////////////////////////////////////////////////////////
///@{

bool TeensyDACGalvo::Busy() {
	//hprot::prot_bool_t moving;
	//TeensyDACHub* hub = static_cast<TeensyDACHub*>(GetParentHub());

	//if (!hub->dispatchGet(CHAN_GET_IS_MOVING, moving)) {
	//	// not busy if error received
	//	return false;
	//}
	//return moving != 0;
	return false;
}

///@}


//////////////////////////////////////////////////////////
/// \name MM::Stage implementation
//////////////////////////////////////////////////////////
///@{

int TeensyDACGalvo::SetPositionUm(double pos) {
	// Treat position in um as wavelength in nm
	return pos_.setProperty(static_cast<position_t>(pos));
}

int TeensyDACGalvo::GetPositionUm(double& pos) {
	// Treat position in um as wavelength in nm
	pos = pos_.getCachedValue();
	return DEVICE_OK;
}

double TeensyDACGalvo::GetStepSize() const {
	// Treat position in um as wavelength in nm
	return POSITION_PRECISION;
}

int TeensyDACGalvo::SetPositionSteps(long steps) {
	// Treat position in um as wavelength in nm
	double pos_nm = steps * POSITION_PRECISION + MIN_POSITION;
	return OnStagePositionChanged(pos_nm);
}

int TeensyDACGalvo::GetPositionSteps(long& steps) {
	position_t pos_voltage = pos_.getCachedValue();
	steps = static_cast<long>((pos_voltage - MIN_POSITION) / POSITION_PRECISION);
	return DEVICE_OK;
}


int TeensyDACGalvo::GetLimits(double& lower, double& upper) {
	// Treat position in um as wavelength in nm
	lower = MIN_POSITION;
	upper = MAX_POSITION;
	return DEVICE_OK;
}

#if defined(GALVOS_SEQUENCABLE)
int TeensyDACGalvo::GetStageSequenceMaxLength(long& nrEvents) const {
	hprot::prot_size_t size;
	int ret;
	if ((ret = pos_.getRemoteSequenceSize(size)) == DEVICE_OK) {
		nrEvents = size;
	}
	return ret;
};

int TeensyDACGalvo::StartStageSequence() {
	int ret = pos_.startRemoteSequence();
	return ret;
}

int TeensyDACGalvo::StopStageSequence() {
	int ret = pos_.stopRemoteSequence();
	return ret;
};

int TeensyDACGalvo::ClearStageSequence() {
	posSequence_.clear();
	return DEVICE_OK;
};

int TeensyDACGalvo::AddToStageSequence(double pos_um) {
	posSequence_.push_back(std::to_string(pos_um));
	return DEVICE_OK;
}

int TeensyDACGalvo::SendStageSequence() {
	if (posSequence_.size() > 0) {
		return pos_.setRemoteSequence(posSequence_);
	}
	return DEVICE_OK;
}
#endif

///@}

#if defined(HAS_SHUTTERS)

//#############################################################################
//### class TeensyDACShutter
//#############################################################################


////////////////////////////////////////////////////////////////
/// \name constructors and destructors
////////////////////////////////////////////////////////////////
///@{
TeensyDACShutter::TeensyDACShutter(channel_t __chan) : chan_(__chan) {
	InitializeDefaultErrorMessages();
	initCommonErrors("Arduino", CURRENT_VERSION, [this](int err, const char* txt) {
		SetErrorText(err, txt);
	});
}

TeensyDACShutter::~TeensyDACShutter() {
	TeensyDACShutter::Shutdown();
}
///@}

//////////////////////////////////////////////////////////
/// \name MM::Device implementation
//////////////////////////////////////////////////////////
///@{
int TeensyDACShutter::Shutdown() {
	return DEVICE_OK;
}

int TeensyDACShutter::Initialize() {
	TeensyDACHub* hub = static_cast<TeensyDACHub*>(GetParentHub());
	if (!hub || !hub->isPortAvailable()) {
		return ERR_NO_PORT_SET;
	}
	try {
		CommandSet cmds = CommandSet::build();	// dummy to initialize

		cmds = CommandSet::build().withChan(chan_).withSet(SH_SET_OPEN).withGet(SH_GET_OPEN)
			.withSetSeq(SH_SETSEQ_OPEN).withGetSeq(SH_GETSEQ_OPEN)
			.withStartSeq(SH_STARTSEQ_OPEN).withStopSeq(SH_STOPSEQ_OPEN);
		assertOK(open_.createRemoteProp(this, hub, g_infoShutterOpen, cmds));

	} catch (DeviceResultException deviceError) {
		LogMessage(deviceError.format(this));
		std::string lastTrans = hub->getLastLog();
		if (!lastTrans.empty()) {
			LogMessage(std::string("Last Transaction: ") + lastTrans);
		}
		return deviceError.error;
	}
	return DEVICE_OK;
}
///@}

//////////////////////////////////////////////////////////
/// \name MM::Shutter implementation
//////////////////////////////////////////////////////////
///@{

bool TeensyDACShutter::Busy() {
	//hprot::prot_bool_t moving;
	//TeensyDACHub* hub = static_cast<TeensyDACHub*>(GetParentHub());
	//if (!hub->dispatchGet(CHAN_GET_IS_MOVING, moving)) {
	//	// not busy if error received
	//	return false;
	//}
	//return moving != 0;
	return false;
}

int TeensyDACShutter::SetOpen(bool __open) {
	open_.setProperty(__open ? 1 : 0);
	return DEVICE_OK;
}

int TeensyDACShutter::GetOpen(bool& __open) {
	hprot::prot_bool_t op;
	open_.getProperty(op);
	__open = (op != 0);
	return DEVICE_OK;
}
///@}

#endif // HAS_SHUTTERS

///@}
