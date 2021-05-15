#pragma once

#include <cstdint>
#include "HexProtocol.h"

namespace tdac {
	typedef hprot::prot_chan_t channel_t;
	typedef hprot::prot_float_t position_t;
	typedef ::std::uint16_t version_t;

    #define FIRMWARE_STR	"KITeensyDAC"
	const unsigned long BAUDRATE = 115200;
	const version_t CURRENT_VERSION = 1;
	const channel_t MAX_NUM_GALVOS = 4;
	const channel_t MAX_NUM_SHUTTERS = 4;
	const double MAX_POSITION = 10.0;
	const double MIN_POSITION = -10.0;
	const long PRECISION_BITS = 16;
	const double POSITION_RANGE = MAX_POSITION - MIN_POSITION;
	const double POSITION_PRECISION = POSITION_RANGE / (1 << PRECISION_BITS);


	/** Github action version string **/
	#define GIT_REVISION_STR	""
	/** Default revision number if SVN_REVISION_STR is malformed*/
    #define DEFAULT_REVISION	100

	/** Deterine the revision number from the subversion auto-props string.
	I have tried to make it as safe as possible. */
	const version_t CURRENT_REVISION = static_cast<version_t>(
		strchr(GIT_REVISION_STR, ':') ?
			strtol(strchr(GIT_REVISION_STR, ':') + 1, 0, 10)
			: DEFAULT_REVISION
	);


	enum Commands {
		// global data
		GET_IS_ALIVE = 'A',
		GET_FIRMWARE,
		GET_VERSION,
		GET_REVISION,
		GET_NUM_GALVOS,
		GET_NUM_SHUTTERS,
		TASK_SHUTDOWN,
		// galvo commands
		GAL_SET_POS,
		GAL_GET_POS,
		GAL_SETSEQ_POS, GAL_GETSEQ_POS, GAL_STARTSEQ_POS, GAL_STOPSEQ_POS, GAL_GETSEQARRAY_POS,
		GAL_GET_ARRAYSEQ_POS,
		GAL_SET_WAVEFORM,
		GAL_GET_WAVEFORM,
		// shutter commands
		SH_SET_OPEN,
		SH_GET_OPEN,
		SH_SETSEQ_OPEN, SH_GETSEQ_OPEN, SH_STARTSEQ_OPEN, SH_STOPSEQ_OPEN, SH_GETSEQARRAY_OPEN
	};
}