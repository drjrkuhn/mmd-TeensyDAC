// for dtostre & dtostrf
#include "AD57X4R.h"
#include "TeensyDACProtocol.h"
#include "utility/dspinst.h"
#include <Arduino.h>
#include <ArduinoStreamHexProtocol.h>
#include <Bounce2.h>
#include <HexProtocol.h>

#define DEBUG_LEVEL 0

/** Sine wave generation is copied from the Teensy Audio Library for now */
#define LOOP_FREQ_HZ 40'000
#define LOOP_DELAY_USEC (1'000'000 / LOOP_FREQ_HZ) // should be 25 usec for 40 kHz

#define AUDIO_SAMPLE_RATE_EXACT (float(LOOP_FREQ_HZ))
#define AUDIO_BLOCK_SAMPLES 256
//#include <Audio.h>

using namespace hprot;
using namespace tdac;

#define BUTTON1_PIN 2
#define CHIP_SELECT_PIN 10
#define LOAD_DAC_PIN 3
#define CLEAR_PIN 4

// #if !defined(__ARM_ARCH_7EM__)
// #error "Architecture not supported yet"
// #endif

// FROM THE TEENSY AUDIO LIBRARY
// These audio waveforms have a period of 256 points, plus a 257th
// point that is a duplicate of the first point.  This duplicate
// is needed because the waveform generator uses linear interpolation
// between each point and the next point in the waveform.
const int16_t AudioWaveformSine[257] = {
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179,
    7962, 8739, 9512, 10278, 11039, 11793, 12539, 13279, 14010, 14732,
    15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403,
    22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790,
    27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571,
    30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521,
    32609, 32678, 32728, 32757, 32767, 32757, 32728, 32678, 32609, 32521,
    32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
    30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790,
    26319, 25832, 25329, 24811, 24279, 23731, 23170, 22594, 22005, 21403,
    20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732,
    14010, 13279, 12539, 11793, 11039, 10278, 9512, 8739, 7962, 7179,
    6393, 5602, 4808, 4011, 3212, 2410, 1608, 804, 0, -804,
    -1608, -2410, -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739,
    -9512, -10278, -11039, -11793, -12539, -13279, -14010, -14732, -15446, -16151,
    -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
    -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683,
    -28105, -28510, -28898, -29268, -29621, -29956, -30273, -30571, -30852, -31113,
    -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609, -32678,
    -32728, -32757, -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285,
    -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571, -30273, -29956,
    -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832,
    -25329, -24811, -24279, -23731, -23170, -22594, -22005, -21403, -20787, -20159,
    -19519, -18868, -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
    -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179, -6393, -5602,
    -4808, -4011, -3212, -2410, -1608, -804, 0};

AD57X4R dac = AD57X4R(CHIP_SELECT_PIN);

const char g_firmwareString[] PROGMEM = FIRMWARE_STR;
version_t g_version                   = CURRENT_VERSION;
version_t g_revision                  = CURRENT_REVISION;

// will keep track of outputs globally for now
class SineWaveform
{
 protected:
    size_t chan_;
    bool isConstant_;
    float frequency_;
    uint32_t phase_accumulator_;
    uint32_t phase_increment_;
    int32_t offset_;
    int32_t magnitude_;
    int32_t position_;
    float voltage_;

 public:
    SineWaveform(channel_t __chan)
    {
        chan_              = __chan;
        isConstant_        = true;
        frequency_         = 0.0f;
        phase_accumulator_ = 0;
        phase_increment_   = 0;
        offset_            = 0;
        magnitude_         = 0;
        position_          = 0;
        voltage_           = 0.0f;
    }

    size_t getChannel()
    {
        return chan_;
    }

    float getVoltage()
    {
        if (isConstant_)
        {
            return dac.analogValueToVoltage(chan_, position_);
        }
        else
        {
            return dac.analogValueToVoltage(chan_, magnitude_ / 2);
        }
    }

    void setVoltage(float voltage)
    {
        if (isConstant_)
        {
            position_    = dac.voltageToAnalogValue(chan_, voltage);
            magnitude_ = 0;
        }
        else
        {
            position_    = dac.voltageToAnalogValue(chan_, 0.0f);
            magnitude_ = 2*dac.voltageToAnalogValue(chan_, voltage);
        }
        voltage_ = voltage;
    }

    float getOffset()
    {
        return dac.analogValueToVoltage(chan_, offset_);
    }

    void setOffset(float offset)
    {
        offset_ = dac.voltageToAnalogValue(chan_, offset);
    }

    void setFrequency(float freqHz)
    {
        if (freqHz < 0.0f) {
            freqHz = 0.0f;
        } else if (freqHz > AUDIO_SAMPLE_RATE_EXACT / 2) {
            freqHz = AUDIO_SAMPLE_RATE_EXACT / 2;
        }
        if (freqHz <= 1e-3f)
        {
            // near zero is considered constant
            isConstant_ = true;
            frequency_  = 0;
            setVoltage(voltage_);
        }
        else
        {
            // multipler 4294967296 = 256 * 2^24
            phase_increment_ = freqHz * (4294967296.0 / AUDIO_SAMPLE_RATE_EXACT);
            isConstant_      = false;
            frequency_       = freqHz;
            setVoltage(voltage_);
        }
    }

    float getFrequency()
    {
        return frequency_;
    }

    long updateNextValue()
    {
        if (isConstant_)
        {
            return position_ + offset_;
        }
        // FROM THE TEENSY AUDIO LIBRARY, synth_sine.cpp
        uint32_t index, scale;
        int32_t val1, val2;
        int16_t output;

        // phase_accumulator is a 32-bit value
        // iiiiiiii'ffffffff'ffffffff'xxxxxxxx

        index = phase_accumulator_ >> 24;           // top 8 bits of phase iiiiiiii
        val1  = AudioWaveformSine[index];           // y0
        val2  = AudioWaveformSine[index + 1];       // y1
        scale = (phase_accumulator_ >> 8) & 0xFFFF; // next 16 bits of phase ffffffff'ffffffff, fraction
        val2 *= scale;                              // y1 = y1 * ffffffff'ffffffff
        val1 *= 0x10000 - scale;                    // y0 = y0 * 1'00000000'00000000  - y0 * ffffffff'ffffffff
        //sum = y1*f + y0(1 - f)
        output = multiply_32x32_rshift32(val1 + val2, magnitude_) + position_;

        phase_accumulator_ += phase_increment_;
        return output + offset_;
    }
};

SineWaveform g_dacOuts[MAX_NUM_GALVOS] = {SineWaveform(0), SineWaveform(1), SineWaveform(2), SineWaveform(3)};
int g_numDacs                          = 4;

//////////////////////////////////////////////////////////////////////////////
/// TeensyDAC Stream Hex Protocol Handler
///
class TeensyDACHandler : public ArduinoStreamHexProtocol<TeensyDACHandler>
{
 protected:
 public:
    TeensyDACHandler() {}

    virtual ~TeensyDACHandler() {}

    /** Start transmitting to the host */
    void beginProtocol(Stream* __stream)
    {
        HexProtocolBase::startProtocol(this, __stream);
    }

    void setup()
    {
#if DEBUG_LEVEL >= 2
        Serial.print("\tInitialized DAC");
        Serial.println();
#endif
        dac.setLoadDacPin(LOAD_DAC_PIN);
        dac.setClearPin(CLEAR_PIN);
        dac.setup(AD57X4R::AD5754R);
        delay(100);
        for (int ch = 0; ch < 4; ch++)
        {
            dac.setOutputRange(ch, AD57X4R::BIPOLAR_10V);
        }
        delay(100);
        for (int ch = 0; ch < 4; ch++)
        {
            dac.setVoltage(ch, 0);
        }
    }

    bool isAlive()
    {
        return true;
    }

    void update()
    {
        dac.beginSimultaneousUpdate();
        for (int i = 0; i < g_numDacs; i++)
        {
            long value = g_dacOuts[i].updateNextValue();
            dac.setAnalogValue(g_dacOuts[i].getChannel(), value);
        }
        dac.simultaneousUpdate();
    }

    bool doShutdown()
    {
        // g_shutA.forceClose();
        // g_shutB.forceClose();
        return true;
    }

 protected:
    bool doGetIsAlive(prot_bool_t& __alive)
    {
        __alive = true;
        return true;
    }

    bool doSetPosition(channel_t __chan, position_t __voltage)
    {
        if (__chan >= g_numDacs)
            return false;
        g_dacOuts[__chan].setVoltage(__voltage);
        return true;
    }

    bool doGetPosition(channel_t __chan, position_t& __voltage)
    {
        if (__chan >= g_numDacs)
            return false;
        __voltage = g_dacOuts[__chan].getVoltage();
        return true;
    }

    bool doSetOffset(channel_t __chan, position_t __offset)
    {
        if (__chan >= g_numDacs)
            return false;
        g_dacOuts[__chan].setOffset(__offset);
        return true;
    }

    bool doGetOffset(channel_t __chan, position_t& __offset)
    {
        if (__chan >= g_numDacs)
            return false;
        __offset = g_dacOuts[__chan].getOffset();
        return true;
    }

    bool doSetFrequency(channel_t __chan, frequency_t __freq)
    {
        if (__chan >= g_numDacs)
            return false;
        g_dacOuts[__chan].setFrequency(__freq);
        return true;
    }

    bool doGetFrequency(channel_t __chan, frequency_t& __freq)
    {
        if (__chan >= g_numDacs)
            return false;
        __freq = g_dacOuts[__chan].getFrequency();
        return true;
    }

 public:
    void doProcessCommand(prot_cmd_t cmd)
    {
        switch (cmd)
        {
            case GET_IS_ALIVE:
                processGet<prot_bool_t>(cmd, &TeensyDACHandler::doGetIsAlive);
                break;
            case GET_FIRMWARE:
                processGetString_P(cmd, g_firmwareString);
                break;
            case GET_VERSION:
                processGet(cmd, g_version);
                break;
            case GET_REVISION:
                processGet(cmd, g_revision);
                break;
            case GET_NUM_GALVOS:
                processGet<prot_chan_t>(cmd, MAX_NUM_GALVOS);
                break;
            case TASK_SHUTDOWN:
                processTask(cmd, &TeensyDACHandler::doShutdown);
                break;
            case GAL_SET_POS:
                processChannelSet<position_t>(cmd, &TeensyDACHandler::doSetPosition);
                break;
            case GAL_GET_POS:
                processChannelGet<position_t>(cmd, &TeensyDACHandler::doGetPosition);
                break;
            case GAL_SET_OFF:
                processChannelSet<position_t>(cmd, &TeensyDACHandler::doSetOffset);
                break;
            case GAL_GET_OFF:
                processChannelGet<position_t>(cmd, &TeensyDACHandler::doGetOffset);
                break;
            case GAL_SET_FREQ:
                processChannelSet<frequency_t>(cmd, &TeensyDACHandler::doSetFrequency);
                break;
            case GAL_GET_FREQ:
                processChannelGet<frequency_t>(cmd, &TeensyDACHandler::doGetFrequency);
                break;
            default:
                replyError();
        }
    }
};

TeensyDACHandler g_handler;

//==========================================================================
// Arduino Entry Point
//==========================================================================

void printDacDiagnosis()
{
    int chipCount    = dac.getChipCount();
    int channelCount = dac.getChannelCount();
    // Serial.print("Chip count");
    // Serial.println(chipCount);
    // Serial.print("Channel count");
    // Serial.println(channelCount);

    Serial.println("dac\tpow\tShD");
    for (int i = 0; i < chipCount; i++)
    {
        Serial.print(i);
        Serial.print('\t');
        Serial.print(dac.referencePoweredUp(i));
        Serial.print('\t');
        Serial.println(dac.thermalShutdown(i));
    }

    Serial.println("ch\tpow\tOvA");
    for (int i = 0; i < channelCount; i++)
    {
        Serial.print(i);
        Serial.print('\t');
        Serial.print(dac.channelPoweredUp(i));
        Serial.print('\t');
        Serial.println(dac.channelOverCurrent(i));
    }
}

// the setup function runs once when you press reset or power the board
void setup()
{
    Serial.begin(BAUDRATE);
#if DEBUG_LEVEL >= 1
    delay(5000);
    Serial.print("Starting Firmware version ");
    Serial.println(g_firmwareString);
#endif
    // Serial.setTimeout(g_serialTimeOut);

#if DEBUG_LEVEL >= 1
    Serial.println("Setup TeensyDACHandler");
#endif

    // do any setup routines
    g_handler.setup();

#if DEBUG_LEVEL >= 1
    printDacDiagnosis();
#endif

#if DEBUG_LEVEL >= 1
    Serial.println("Starting HexProtocol");
#endif
    // start listening for messages from MMDevice driver
    g_handler.beginProtocol(&Serial);
}

// the loop function runs over and over again until power down or reset
void loop()
{
    unsigned long starttime, endtime;
    starttime = micros();

    // then look for new commands
    if (g_handler.hasCommand())
    {
        g_handler.processCommand(g_handler.getCommand(), &TeensyDACHandler::doProcessCommand);
    }

    // 	bool powerSense = digitalRead(POWER_SENSE_PIN);
    // 	if (powerSense != g_handler.isAlive()) {
    // #if DEBUG_LEVEL >= 1
    // 		Serial.print("Setup powerToggled: ");
    // 		Serial.print(powerSense ? "On" : "Off");
    // 		Serial.println();
    // #endif
    // 		// toggle power
    // 		if (powerSense) {
    // 			g_handler.powerOn();
    // 		} else {
    // 			g_handler.powerOff();
    // 		}
    // 	}

    // update the outputs
    g_handler.update();

    endtime = micros();
    unsigned long spenttime = (endtime - starttime);
    long nextdelay = LOOP_DELAY_USEC - spenttime;
    if (nextdelay > 0) {
        delayMicroseconds(nextdelay);
    }
}
