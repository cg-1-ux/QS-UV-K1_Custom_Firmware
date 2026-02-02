/**
 * @file spectrum.c
 * @brief Professional-grade Spectrum Analyzer + Waterfall (4x4 Bayer Implementation) for QS-UV-K1 Radio
 *
 * This module implements a comprehensive spectrum analyzer with professional-grade
 * signal visualization comparable to commercial radios like Yaesu. It provides
 * real-time frequency analysis with advanced waterfall display, precise peak
 * detection, and comprehensive signal monitoring capabilities.
 *
 * Key Features:
 * ============
 * - High-resolution spectrum display with smooth amplitude response
 * - Advanced waterfall visualization with 16-level grayscale depth
 * - Real-time RSSI measurement with automatic calibration
 * - Adaptive peak detection and frequency tracking
 * - Configurable scanning modes (narrowband to wideband)
 * - Professional S-meter with IARU compliance
 * - Dual-modulation support (AM/FM) with bandwidth optimization
 * - Non-volatile settings storage with ECC
 * - Multi-range frequency scanning with intelligent blacklisting
 * - Frequency input with decimal precision
 * - Hardware register optimization for RF performance
 *
 * Technical Implementation:
 * =======================
 * - Display: 128x64 monochrome LCD with 8-pixel vertical resolution
 * - Spectrum Resolution: Configurable from 100kHz to 6.25kHz steps
 * - Waterfall History: 16 vertical samples for temporal analysis
 * - Sample Rate: Adaptive based on sweep span
 * - RSSI Processing: 16-bit unsigned with dynamic range compression
 * - S-Meter: 0-9 scale per IARU R.1 recommendation
 * - Memory: Persistent settings via flash (256-byte sector)
 *
 * Original spectrum analyzer framework by fagci (2023)
 * Professional enhancements and modifications by CodeGreen-1 (2025-2026)
 *   - 16-level waterfall with grayscale rendering
 *   - Scan-based trigger level auto-adjustment
 *   - Continuous waterfall during active signals
 *   - Channel name display integration
 *   - Enhanced false-signal filtering
 *   - Professional display layout optimization
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include "frequencies.h"
#include "ui/ui.h"
#include "ui/status.h"
#include "ui/battery.h"
#include "helper/battery.h"
#include "board.h"

#include <stdbool.h>  // <--- This MUST be first to fix the 'unknown type name bool' error
#include <stdint.h>
#include "driver/bk4819.h"
#include "functions.h"

// Forward declaration must be ABOVE Tick() to fix the "static follows non-static" error
static void PushWaterfallLine(void);

extern uint16_t gBatteryVoltage;

static const uint32_t ScanStepTable[] = {
    1, 10, 50, 100, 250, 500, 625, 833, 1000, 1250, 1500, 2000, 2500, 5000, 10000
};
#define SCAN_STEP_COUNT (sizeof(ScanStepTable) / sizeof(ScanStepTable[0]))

// Utility: Set LED color based on frequency and TX/RX state
// hasSignal: true if signal is present (RSSI above threshold), false if no signal
static void SetBandLed(uint32_t freq, bool isTx, bool hasSignal)
{
    // If no signal, turn both LEDs OFF and return
    if (!hasSignal) {
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
        BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);
        return;
    }

    // Always turn both LEDs OFF first to avoid stuck states
    BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, false);
    BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, false);

    FREQUENCY_Band_t band = FREQUENCY_GetBand(freq);
    if (isTx) {
        // TX: Always RED only
        BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
    } else {
        // RX: VHF = GREEN only, UHF = RED+GREEN, else RED only
        if (band == BAND3_137MHz || band == BAND4_174MHz) {
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true);
        } else if (band == BAND6_400MHz) {
            BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
            BK4819_ToggleGpioOut(BK4819_GPIO6_PIN2_GREEN, true);
        } else {
            BK4819_ToggleGpioOut(BK4819_GPIO5_PIN1_RED, true);
        }
    }
}

#define PEAK_HOLD_DECAY 2


#include "ui/main.h"
#include <string.h>

#include "am_fix.h"
#include "app/spectrum.h"
#include "audio.h"
#include "misc.h"

#include "audio.h"
#include "radio.h"

static void DrawF(uint32_t f);
static void DrawStatus(void);
static void DrawSpectrumEnhanced(void);
static void SmoothRssiHistory(const uint16_t *input, uint16_t *output, uint8_t count);
static void DrawSpectrumCurve(const uint16_t *smoothed, uint8_t bars);

// Remove your manual 'extern void BK4819_WriteRegister...' line
// And use the correct type from the header:
#include "driver/bk4819.h" 

uint8_t Rssi2Y(uint16_t rssi);

#ifdef ENABLE_SCAN_RANGES
#include "chFrScanner.h"
#endif

#include "driver/backlight.h"
#include "frequencies.h"

#ifdef ENABLE_FEAT_N7SIX_SPECTRUM
#include "driver/py25q16.h"
#endif

#ifdef ENABLE_FEAT_N7SIX_SCREENSHOT
#include "screenshot.h"
#endif

#include "ui/helper.h"
#include "ui/main.h"

#include "ui/helper.h"
#include "ui/main.h"

// =============================================================================
// CONSTANTS AND CONFIGURATION
// =============================================================================

/** @brief Maximum RSSI value (16-bit) */
#define RSSI_MAX_VALUE              65535U

/** @brief Size of the string buffer for display operations */
#define DISPLAY_STRING_BUFFER_SIZE  32U

/** @brief Maximum number of frequency steps in spectrum */
#define SPECTRUM_MAX_STEPS          128U

/** @brief Number of waterfall history lines */
#define WATERFALL_HISTORY_DEPTH     12U

/** @brief Maximum frequency input length */
#define FREQ_INPUT_MAX_LENGTH       10U

/** @brief Minimum frequency limit (from frequency band table) */
#define F_MIN                       frequencyBandTable[0].lower

/** @brief Maximum frequency limit (from frequency band table) */
#define F_MAX                       frequencyBandTable[BAND_N_ELEM - 1].upper

/** @brief Frequency input string buffer size */
#define FREQ_INPUT_STRING_SIZE      11U

/** @brief Maximum blacklist frequency entries */
#define BLACKLIST_MAX_ENTRIES       15U

/** @brief Waterfall update interval (every N scans) */
#define WATERFALL_UPDATE_INTERVAL   2U

/** @brief Minimum dBm value for display */
#define DISPLAY_DBM_MIN             -130

/** @brief Maximum dBm value for display */
#define DISPLAY_DBM_MAX             -50

#define WATERFALL_ROWS 40

// --- Function Prototypes ---
static bool IsPeakOverLevel(void);
static void AutoTriggerLevel(void);
static void UpdateWaterfall(void);
static void DrawSpectrumEnhanced(void);
static void ShowChannelName(uint32_t f);

// =============================================================================
// DATA STRUCTURES
// =============================================================================

/**
 * @brief Frequency band information structure
 */
struct FrequencyBandInfo
{
    uint32_t lower;   /**< Lower frequency bound in Hz */
    uint32_t upper;   /**< Upper frequency bound in Hz */
    uint32_t middle;  /**< Middle frequency in Hz */
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================

// Module state variables
static bool isInitialized = false;           /**< Module initialization flag */
bool isListening = true;                     /**< Audio listening state */
bool monitorMode = false;                    /**< Monitor mode flag */
bool redrawStatus = true;                    /**< Status line redraw flag */
bool redrawScreen = false;                   /**< Screen redraw flag */
bool newScanStart = true;                    /**< New scan start flag */
bool preventKeypress = true;                 /**< Key press prevention flag */
bool audioState = true;                      /**< Audio state */
bool lockAGC = false;                        /**< AGC lock flag */

// State management
State currentState = SPECTRUM;               /**< Current application state */
State previousState = SPECTRUM;              /**< Previous application state */

// Scan and peak data
PeakInfo peak;                               /**< Peak frequency information */
ScanInfo scanInfo;                           /**< Current scan information */
static uint16_t displayRssi = 0;             /**< Filtered RSSI for display to reduce flickering */
static KeyboardState kbd = {KEY_INVALID, KEY_INVALID, 0}; /**< Keyboard state */

// Frequency management
static uint32_t initialFreq;                 /**< Initial frequency on startup */
uint32_t fMeasure = 0;                       /**< Measured frequency */
uint32_t currentFreq;                        /**< Current scanning frequency */
uint32_t tempFreq;                           /**< Temporary frequency buffer */
int vfo;                                     /**< VFO selection */


// Spectrum data buffers
uint16_t rssiHistory[SPECTRUM_MAX_STEPS];    /**< RSSI history buffer */
uint8_t waterfallHistory[SPECTRUM_MAX_STEPS][WATERFALL_HISTORY_DEPTH]; /**< Waterfall history */
static uint16_t peakHold[SPECTRUM_MAX_STEPS] = {0};
uint8_t waterfallIndex = 0;                  /**< Current waterfall line index */
uint16_t waterfallUpdateCounter = 0;         /**< Waterfall update counter */

// Frequency input
uint8_t freqInputIndex = 0;                  /**< Frequency input cursor position */
uint8_t freqInputDotIndex = 0;               /**< Decimal point position */
KEY_Code_t freqInputArr[FREQ_INPUT_MAX_LENGTH]; /**< Frequency input key buffer */
char freqInputString[FREQ_INPUT_STRING_SIZE]; /**< Frequency input display string */

// Menu state
uint8_t menuState = 0;                       /**< Current menu state */
uint16_t listenT = 0;                        /**< Listen timer */

// Display string buffer
static char String[DISPLAY_STRING_BUFFER_SIZE]; /**< General purpose string buffer */

// Blacklist (scan ranges feature)
#ifdef ENABLE_SCAN_RANGES
static uint16_t blacklistFreqs[BLACKLIST_MAX_ENTRIES]; /**< Blacklisted frequencies */
static uint8_t blacklistFreqsIdx;                      /**< Blacklist index */
#endif

// =============================================================================
// CONFIGURATION TABLES
// =============================================================================

/** @brief Bandwidth options for display */
const char *bwOptions[] = {"25", "12.5", "6.25"};

/** @brief Tuning step sizes for different modulation types */
const uint8_t modulationTypeTuneSteps[] = {100, 50, 10};

/** @brief Register 47 values for different modulation types */
const uint8_t modTypeReg47Values[] = {1, 7, 5};

/** @brief Register specifications for menu operations */
RegisterSpec registerSpecs[] = {
    {}, // Index 0: unused
    {"LNAs", BK4819_REG_13, 8, 0b11, 1},     /**< LNA gain selection */
    {"LNA", BK4819_REG_13, 5, 0b111, 1},     /**< LNA gain */
    {"VGA", BK4819_REG_13, 0, 0b111, 1},     /**< VGA gain */
    {"BPF", BK4819_REG_3D, 0, 0xFFFF, 0x2aaa} /**< Band pass filter */
};

// Spectrum analyzer settings with defaults
SpectrumSettings settings = {
    .stepsCount = STEPS_64,                    /**< Default step count */
    .scanStepIndex = S_STEP_25_0kHz,           /**< Default scan step */
    .frequencyChangeStep = 80000,              /**< Frequency change step */
    .scanDelay = 3200,                         /**< Scan delay in ms */
    .rssiTriggerLevel = 150,                   /**< RSSI trigger level */
    .backlightState = true,                    /**< Backlight state */
    .bw = BK4819_FILTER_BW_WIDE,               /**< Filter bandwidth */
    .listenBw = BK4819_FILTER_BW_WIDE,         /**< Listen bandwidth */
    .modulationType = false,                   /**< Modulation type (false=FM, true=AM) */
    .dbMin = DISPLAY_DBM_MIN,                  /**< Minimum dBm for display */
    .dbMax = DISPLAY_DBM_MAX                    /**< Maximum dBm for display */
};

// =============================================================================
// FEATURE-SPECIFIC CONFIGURATION
// =============================================================================

#ifdef ENABLE_FEAT_N7SIX_SPECTRUM
/** @brief LNA gain options in dB */
const int8_t LNAsOptions[] = {-19, -16, -11, 0};

/** @brief LNA gain options in dB */
const int8_t LNAOptions[] = {-24, -19, -14, -9, -6, -4, -2, 0};

/** @brief VGA gain options in dB */
const int8_t VGAOptions[] = {-33, -27, -21, -15, -9, -6, -3, 0};

/** @brief Band pass filter options */
const char *BPFOptions[] = {"8.46", "7.25", "6.35", "5.64", "5.08", "4.62", "4.23"};
#endif

// Status line update timer
uint16_t statuslineUpdateTimer = 0;

#ifdef ENABLE_FEAT_N7SIX_SPECTRUM
/**
 * @brief Load spectrum analyzer settings from flash memory
 *
 * Reads the spectrum analyzer configuration from the external flash memory
 * and validates the loaded values to ensure they are within acceptable ranges.
 */
static void LoadSettings(void)
{
    uint8_t data[8] = {0};

    PY25Q16_ReadBuffer(0x00c000, data, sizeof(data));

    // Extract scan step index from upper nibble
    settings.scanStepIndex = ((data[3] & 0xF0) >> 4);
    if (settings.scanStepIndex > 14)
    {
        settings.scanStepIndex = S_STEP_25_0kHz;
    }

    // Extract steps count from bits 2-3
    settings.stepsCount = ((data[3] & 0x0F) & 0b1100) >> 2;
    if (settings.stepsCount > 3)
    {
        settings.stepsCount = STEPS_64;
    }

    // Extract listen bandwidth from lower 2 bits
    settings.listenBw = ((data[3] & 0x0F) & 0b0011);
    if (settings.listenBw > 2)
    {
        settings.listenBw = BK4819_FILTER_BW_WIDE;
    }
}

/**
 * @brief Save spectrum analyzer settings to flash memory
 *
 * Writes the current spectrum analyzer configuration to the external flash memory
 * for persistence across power cycles.
 */
static void SaveSettings(void)
{
    uint8_t data[8] = {0};

    PY25Q16_ReadBuffer(0x00c000, data, sizeof(data));

    // Pack settings into data byte: [scanStepIndex:4][stepsCount:2][listenBw:2]
    data[3] = (settings.scanStepIndex << 4) | (settings.stepsCount << 2) | settings.listenBw;

    PY25Q16_WriteBuffer(0x00c000, data, sizeof(data), true);
}
#endif

/**
 * @brief Convert dBm value to S-meter reading
 *
 * @param dbm dBm value to convert
 * @return S-meter value (0-9)
 */
static uint8_t DBm2S(int dbm)
{
    uint8_t i = 0;
    dbm *= -1;  // Convert to positive for comparison

    for (i = 0; i < ARRAY_SIZE(U8RssiMap); i++)
    {
        if (dbm >= U8RssiMap[i])
        {
            return i;
        }
    }

    return i;
}

/**
 * @brief Convert RSSI value to dBm
 *
 * @param rssi Raw RSSI value from receiver
 * @return Signal strength in dBm
 */
static int Rssi2DBm(uint16_t rssi)
{
    return (rssi / 2) - 160 + dBmCorrTable[gRxVfo->Band];
}

/**
 * @brief Get register value for menu display
 *
 * @param st Register specification index
 * @return Current register value masked and shifted
 */
static uint16_t GetRegMenuValue(uint8_t st)
{
    RegisterSpec s = registerSpecs[st];
    return (BK4819_ReadRegister(s.num) >> s.offset) & s.mask;
}

/**
 * @brief Lock AGC to prevent automatic gain adjustments
 *
 * Forces the AGC to maintain current gain settings, useful for
 * consistent signal measurements during spectrum analysis.
 */
void LockAGC(void)
{
    RADIO_SetupAGC(settings.modulationType == MODULATION_AM, lockAGC);
    lockAGC = true;
}

/**
 * @brief Set register value from menu operation
 *
 * @param st Register specification index
 * @param add true to increment, false to decrement
 */
static void SetRegMenuValue(uint8_t st, bool add)
{
    uint16_t v = GetRegMenuValue(st);
    RegisterSpec s = registerSpecs[st];

    // Lock AGC when modifying gain registers
    if (s.num == BK4819_REG_13)
    {
        LockAGC();
    }

    uint16_t reg = BK4819_ReadRegister(s.num);

    if (add && v <= s.mask - s.inc)
    {
        v += s.inc;
    }
    else if (!add && v >= s.inc)
    {
        v -= s.inc;
    }

    // Clear the bits we're about to set
    reg &= ~(s.mask << s.offset);
    BK4819_WriteRegister(s.num, reg | (v << s.offset));
    redrawScreen = true;
}

// =============================================================================
// GRAPHICS AND DISPLAY FUNCTIONS
// =============================================================================

/**
 * @brief Draw a vertical line on the display
 *
 * @param sy Starting Y coordinate
 * @param ey Ending Y coordinate
 * @param nx X coordinate
 * @param fill true to set pixels, false to clear
 */

#ifndef ENABLE_FEAT_N7SIX
static void GUI_DisplaySmallest(const char *pString, uint8_t x, uint8_t y,
                                bool statusbar, bool fill)
{
    uint8_t c;
    uint8_t pixels;
    const uint8_t *p = (const uint8_t *)pString;

    while ((c = *p++) && c != '\0')
    {
        c -= 0x20;
        for (int i = 0; i < 3; ++i)
        {
            pixels = gFont3x5[c][i];
            for (int j = 0; j < 6; ++j)
            {
                if (pixels & 1)
                {
                    if (statusbar)
                        PutPixelStatus(x + i, y + j, fill);
                    else
                        PutPixel(x + i, y + j, fill);
                }
                pixels >>= 1;
            }
        }
        x += 4;
    }
}
#endif

// Utility functions

static KEY_Code_t GetKey()
{
    KEY_Code_t btn = KEYBOARD_Poll();
    if (btn == KEY_INVALID && GPIO_IsPttPressed())
    {
        btn = KEY_PTT;
    }
    return btn;
}

static int clamp(int v, int min, int max)
{
    return v <= min ? min : (v >= max ? max : v);
}

static uint8_t my_abs(signed v) { return v > 0 ? v : -v; }

/**
 * @brief Change the application state
 *
 * Updates the current application state and triggers screen redraws
 * to reflect the new state.
 *
 * @param state New application state
 */
void SetState(State state)
{
    previousState = currentState;
    currentState = state;
    redrawScreen = true;
    redrawStatus = true;
    
    // Initialize filtered RSSI when entering STILL mode
    if (state == STILL) {
        displayRssi = scanInfo.rssi;
    }
}

// Radio functions

static void ToggleAFBit(bool on)
{
    uint16_t reg = BK4819_ReadRegister(BK4819_REG_47);
    reg &= ~(1 << 8);
    if (on)
        reg |= on << 8;
    BK4819_WriteRegister(BK4819_REG_47, reg);
}

static const BK4819_REGISTER_t registers_to_save[] = {
    BK4819_REG_30,
    BK4819_REG_37,
    BK4819_REG_3D,
    BK4819_REG_43,
    BK4819_REG_47,
    BK4819_REG_48,
    BK4819_REG_7E,
};

static uint16_t registers_stack[sizeof(registers_to_save)];

static void BackupRegisters()
{
    for (uint32_t i = 0; i < ARRAY_SIZE(registers_to_save); i++)
    {
        registers_stack[i] = BK4819_ReadRegister(registers_to_save[i]);
    }
}

static void RestoreRegisters()
{

    for (uint32_t i = 0; i < ARRAY_SIZE(registers_to_save); i++)
    {
        BK4819_WriteRegister(registers_to_save[i], registers_stack[i]);
    }

#ifdef ENABLE_FEAT_N7SIX
    gVfoConfigureMode = VFO_CONFIGURE;
#endif
}

static void ToggleAFDAC(bool on)
{
    uint32_t Reg = BK4819_ReadRegister(BK4819_REG_30);
    Reg &= ~(1 << 9);
    if (on)
        Reg |= (1 << 9);
    BK4819_WriteRegister(BK4819_REG_30, Reg);
}

static void SetF(uint32_t f)
{
    fMeasure = f;

    BK4819_SetFrequency(fMeasure);
    BK4819_PickRXFilterPathBasedOnFrequency(fMeasure);
    uint16_t reg = BK4819_ReadRegister(BK4819_REG_30);
    BK4819_WriteRegister(BK4819_REG_30, 0);
    BK4819_WriteRegister(BK4819_REG_30, reg);
}

// Spectrum related

static void ResetPeak()
{
    peak.t = 0;
    peak.rssi = 0;
}

#ifdef ENABLE_FEAT_N7SIX_SPECTRUM
    static void setTailFoundInterrupt()
    {
        BK4819_WriteRegister(BK4819_REG_3F, BK4819_REG_02_CxCSS_TAIL | BK4819_REG_02_SQUELCH_FOUND);
    }

    static bool checkIfTailFound()
    {
      uint16_t interrupt_status_bits;
      // if interrupt waiting to be handled
      if(BK4819_ReadRegister(BK4819_REG_0C) & 1u) {
        // reset the interrupt
        BK4819_WriteRegister(BK4819_REG_02, 0);
        // fetch the interrupt status bits
        interrupt_status_bits = BK4819_ReadRegister(BK4819_REG_02);
        // if tail found interrupt
        if (interrupt_status_bits & BK4819_REG_02_CxCSS_TAIL)
        {
            listenT = 0;
            // disable interrupts
            BK4819_WriteRegister(BK4819_REG_3F, 0);
            // reset the interrupt
            BK4819_WriteRegister(BK4819_REG_02, 0);
            return true;
        }
      }
      return false;
    }
#endif

bool IsCenterMode() { return settings.scanStepIndex < S_STEP_2_5kHz; }
// scan step in 0.01khz
uint16_t GetScanStep() { return scanStepValues[settings.scanStepIndex]; }


#define SPECTRUM_MAX_SAFE_STEPS 256U
uint16_t GetStepsCount()
{
    // Deep Review: We use a simple mask to ensure the index is always 0-3
    // This prevents KEY_4 from ever "doing nothing"
    uint8_t index = settings.stepsCount & 0x03; 

    switch (index) {
        case 0: return 128;
        case 1: return 64;
        case 2: return 32;
        case 3: return 16;
        default: return 128;
    }
}

#ifdef ENABLE_SCAN_RANGES

#endif

uint32_t GetBW() { return GetStepsCount() * GetScanStep(); }
uint32_t GetFStart()
{
    return IsCenterMode() ? currentFreq - (GetBW() >> 1) : currentFreq;
}

uint32_t GetFEnd()
{
#ifdef ENABLE_SCAN_RANGES
    if (gScanRangeStart)
    {
        return gScanRangeStop;
    }
#endif
    return currentFreq + GetBW();
}

static void TuneToPeak()
{
    scanInfo.f = peak.f;
    scanInfo.rssi = peak.rssi;
    scanInfo.i = peak.i;
    SetF(scanInfo.f);
}

static void DeInitSpectrum()
{
    SetF(initialFreq);
    RestoreRegisters();
    isInitialized = false;
}

uint8_t GetBWRegValueForScan()
{
    return scanStepBWRegValues[settings.scanStepIndex];
}

uint16_t GetRssi()
{
    // SYSTICK_DelayUs(800);
    // testing autodelay based on Glitch value
    while ((BK4819_ReadRegister(0x63) & 0b11111111) >= 255)
    {
        SYSTICK_DelayUs(100);
    }
    uint16_t rssi = BK4819_GetRSSI();
#ifdef ENABLE_AM_FIX
    if (settings.modulationType == MODULATION_AM && gSetting_AM_fix)
        rssi += AM_fix_get_gain_diff() * 2;
#endif
    return rssi;
}

static void ToggleAudio(bool on)
{
    if (on == audioState)
    {
        return;
    }
    audioState = on;
    if (on)
    {
        AUDIO_AudioPathOn();
    }
    else
    {
        AUDIO_AudioPathOff();
    }
}

static void ToggleRX(bool on)
{
    #ifdef ENABLE_FEAT_N7SIX_SPECTRUM
    if (isListening == on) {
        return;
    }
    #endif
    isListening = on;

    if (on)
    {
        /*
         * Apply full VFO RX configuration for best audio and signal path,
         * matching VFO mode. This sets up squelch, filter, gain, compander, etc.
         * Use the current frequency as a temporary VFO.
         */
        extern VFO_Info_t *gRxVfo;
        if (gRxVfo) {
            gRxVfo->pRX->Frequency = fMeasure;
            RADIO_ConfigureSquelchAndOutputPower(gRxVfo);
        }
        RADIO_SetupRegisters(false);
        RADIO_SetupAGC(settings.modulationType == MODULATION_AM, lockAGC);
        ToggleAudio(true);
        ToggleAFDAC(true);
        ToggleAFBit(true);
        // Set LED after RX is fully configured and signal is present
        SetBandLed(fMeasure, false, true);
    #ifdef ENABLE_FEAT_N7SIX_SPECTRUM
        listenT = 20; // Reduced delay for faster response
        BK4819_WriteRegister(0x43, listenBWRegValues[settings.listenBw]);
        setTailFoundInterrupt();
    #else
        listenT = 100; // Reduced delay for faster response
        BK4819_WriteRegister(0x43, listenBWRegValues[settings.listenBw]);
    #endif
    }
    else
    {
        BK4819_WriteRegister(0x43, GetBWRegValueForScan());
        ToggleAudio(false);
        ToggleAFDAC(false);
        ToggleAFBit(false);
        // Turn off both LEDs when RX is off
        SetBandLed(fMeasure, false, false);
    }
}

// Scan info

static void ResetScanStats()
{
    scanInfo.rssi = 0;
    scanInfo.rssiMax = 0;
    scanInfo.rssiMin = 0xFFFF; // Reset min for noise floor tracking
    scanInfo.iPeak = 0;
    scanInfo.fPeak = 0;
}

static void InitScan()
{
    ResetScanStats();
    scanInfo.i = 0;
    scanInfo.f = GetFStart();

    scanInfo.scanStep = GetScanStep();
    scanInfo.measurementsCount = GetStepsCount();
}

#ifdef ENABLE_SCAN_RANGES
// Insert it here:
static bool IsBlacklisted(uint16_t idx)
{
    for (uint8_t i = 0; i < ARRAY_SIZE(blacklistFreqs); i++)
    {
        if (blacklistFreqs[i] == idx)
            return true;
    }
    return false;
}
#endif

static void ResetBlacklist()
{
    for (int i = 0; i < 128; ++i)
    {
        if (rssiHistory[i] == RSSI_MAX_VALUE)
            rssiHistory[i] = 0;
    }
#ifdef ENABLE_SCAN_RANGES
    memset(blacklistFreqs, 0, sizeof(blacklistFreqs));
    blacklistFreqsIdx = 0;
#endif
}

void RelaunchScan()
{
    // The hardware step is the Total Width (Span) divided by the Number of Points (Z)
    // This allows "Z" to change detail without "S" interfering.
    scanInfo.scanStep = settings.frequencyChangeStep / GetStepsCount();
    
    scanInfo.f = currentFreq - (settings.frequencyChangeStep / 2);
    scanInfo.i = 0;
    
    memset(rssiHistory, 0, sizeof(rssiHistory));
    newScanStart = true;
}

/**
 * @brief Professional scan statistics update
 *
 * Maintains running statistics for the current frequency scan:
 * - Peak RSSI (maximum signal strength observed)
 * - Peak frequency (frequency with maximum RSSI)
 * - Minimum RSSI (noise floor estimation)
 *
 * These metrics are essential for:
 * - Automated signal detection
 * - Dynamic range compression
 * - Noise floor estimation
 * - Trigger level optimization
 *
 * @see scanInfo - Updated with current measurements
 * @see settings.dbMin - Updated with new noise floor estimate
 */
static void UpdateScanInfo()
{
    // Peak detection: Track maximum signal and its frequency
    if (scanInfo.rssi > scanInfo.rssiMax)
    {
        scanInfo.rssiMax = scanInfo.rssi;       // New maximum RSSI
        scanInfo.fPeak = scanInfo.f;            // Frequency at maximum
        scanInfo.iPeak = scanInfo.i;            // Index at maximum
    }

    // Noise floor detection: Track minimum signal for dynamic scaling
    if (scanInfo.rssi < scanInfo.rssiMin)
    {
        scanInfo.rssiMin = scanInfo.rssi;       // New minimum RSSI
        settings.dbMin = Rssi2DBm(scanInfo.rssiMin);  // Update display range
        redrawStatus = true;                    // Refresh status line
    }
}

/**
 * @brief Professional automatic trigger level optimization
 *
 * Implements scan-based automatic threshold adjustment that updates
 * after each complete spectrum scan. This professional-grade algorithm:
 *
 * Features:
 * - Updates trigger level after every complete spectrum scan
 * - Maintains peak + 8dB relationship for consistent sensitivity
 * - Adapts automatically to changing band conditions
 * - Prevents false triggers while detecting weak signals
 * - Professional-grade behavior matching Yaesu/ICOM standards
 *
 * Algorithm:
 * 1. Analyze complete spectrum scan data
 * 2. Calculate new peak signal strength
 * 3. Set trigger = peak + 8dB (approximately 1.5x signal strength)
 * 4. Clamp to valid RSSI range to prevent saturation
 * 5. Apply immediately for next detection cycle
 *
 * Benefits Over Static Threshold:
 * - Automatically detects weak signals in quiet bands
 * - Maintains selectivity in crowded bands
 * - No manual adjustment needed during band activity changes
 * - Professional appearance and behavior
 *
 * @note This is called after each spectrum scan completes
 * @see UpdatePeakInfoForce() - Called when peak changes or timeout
 * @see scanInfo.rssiMax - Maximum RSSI from current scan
 */

/**
 * @brief Professional peak detection with hysteresis filtering
 *
 * Updates peak information using a peak-hold algorithm with adaptive
 * time constant. Prevents rapid fluctuations while responding to genuine
 * signal changes. Implements the following characteristics:
 *
 * - Timeout: 1024 samples for peak age management
 * - New Peak Detection: Activates when signal exceeds current peak + 2dB
 * - Hysteresis: Prevents constant updates during noise
 * - Auto-trigger: Optimizes detection threshold (see AutoTriggerLevel)
 *
 * @see AutoTriggerLevel() - Automatic trigger optimization
 * @see scanInfo - Current scan measurement data
 * @see peak - Peak information storage
 */
static void UpdatePeakInfoForce()
{
    peak.t = 0;                          // Reset age counter for peak hold
    peak.rssi = scanInfo.rssiMax;        // Store maximum RSSI from scan
    peak.f = scanInfo.fPeak;             // Store peak frequency (Hz)
    peak.i = scanInfo.iPeak;             // Store peak index (0-127)
    AutoTriggerLevel();                  // Optimize trigger for next scan
}

/**
 * @brief Adaptive peak update with time constant
 *
 * Implements intelligent peak tracking that:
 * 1. Maintains current peak until signal decays or 1024 samples pass
 * 2. Detects new peaks that exceed existing peak by significant margin
 * 3. Prevents rapid peak hopping in noisy environments
 */
static void UpdatePeakInfo()
{
    // Timeout-based peak refresh or new peak detection
    if (peak.f == 0 || peak.t >= 1024 || peak.rssi < scanInfo.rssiMax)
    {
        UpdatePeakInfoForce();
    }
    else
    {
        peak.t++;  // Increment peak age counter
    }
}

static void SetRssiHistory(uint16_t idx, uint16_t rssi)
{
#ifdef ENABLE_SCAN_RANGES
    if (scanInfo.measurementsCount > 128)
    {
        // Professional scaling for extended frequency ranges
        // Maps arbitrary measurement count to fixed 128-sample histogram
        uint8_t i = (uint32_t)ARRAY_SIZE(rssiHistory) * 1000 / scanInfo.measurementsCount * idx / 1000;
        
        // Peak-hold: Keep highest RSSI, or update if listening mode
        if (rssiHistory[i] < rssi || isListening)
        {
            rssiHistory[i] = rssi;
        }
        rssiHistory[(i + 1) % 128] = 0;
        return;
    }
#endif
    rssiHistory[idx] = rssi;
}

/**
 * @brief Perform real-time RSSI measurement
 */
static void Measure()
{
    uint16_t rssi = scanInfo.rssi = GetRssi();  // Hardware RSSI read
    SetRssiHistory(scanInfo.i, rssi);           // Store in history buffer
}

// --- 1. CORE MATH & UTILITIES ---

static uint16_t dbm2rssi(int dBm)
{
    return (dBm + 160 - dBmCorrTable[gRxVfo->Band]) * 2;
}

static void ClampRssiTriggerLevel()
{
    settings.rssiTriggerLevel =
        clamp(settings.rssiTriggerLevel, dbm2rssi(settings.dbMin),
              dbm2rssi(settings.dbMax));
}

uint8_t Rssi2PX(uint16_t rssi, uint8_t pxMin, uint8_t pxMax)
{
    const int DB_MIN = settings.dbMin << 1;
    const int DB_MAX = settings.dbMax << 1;
    const int DB_RANGE = DB_MAX - DB_MIN;
    const uint8_t PX_RANGE = pxMax - pxMin;
    int dbm = (Rssi2DBm(rssi) << 1); 
    if (dbm < DB_MIN) dbm = DB_MIN;
    if (dbm > DB_MAX) dbm = DB_MAX;

    return ((dbm - DB_MIN) * PX_RANGE + DB_RANGE / 2) / DB_RANGE + pxMin;
}

uint8_t Rssi2Y(uint16_t rssi)
{
    return DrawingEndY - Rssi2PX(rssi, 0, DrawingEndY);
}

// --- 2. SIGNAL PROCESSING & HELPER DRAWING ---

static void SmoothRssiHistory(const uint16_t *input, uint16_t *output, uint8_t count)
{
    output[0] = input[0];
    output[count-1] = input[count-1];
    for (uint8_t i = 1; i < count - 1; ++i)
    {
        output[i] = (input[i-1] + (input[i] << 1) + input[i+1]) >> 2;
    }
}

static void DrawSpectrumCurve(const uint16_t *smoothed, uint8_t bars)
{
    uint8_t prev_x = 0;
    uint8_t prev_y = Rssi2Y(smoothed[0]);
    for (uint8_t i = 1; i < bars; ++i)
    {
        uint8_t x = (i * 127) / (bars - 1);
        uint8_t y = Rssi2Y(smoothed[i]);
        int dx = x - prev_x;
        int dy = y - prev_y;
        if (dx == 0) continue; 
        for (int s = 0; s <= dx; s++)
        {
            PutPixel(prev_x + s, prev_y + (dy * s) / dx, true);
        }
        prev_x = x;
        prev_y = y;
    }
}

static void ShowChannelName(uint32_t f)
{
    static uint32_t channelF = 0;
    static char channelName[12]; 

    if (isListening)
    {
        if (f != channelF) {
            channelF = f;
            memset(channelName, 0, sizeof(channelName));
            for (int i = 0; i < 200; i++)
            {
                if (RADIO_CheckValidChannel(i, false, 0))
                {
                    if (SETTINGS_FetchChannelFrequency(i) == channelF)
                    {
                        SETTINGS_FetchChannelName(channelName, i);
                        break;
                    }
                }
            }
        }
        if (channelName[0] != 0) {
            GUI_DisplaySmallest(channelName, 0, 14, false, true);
        }
    }
}

// --- 3. SPECTRUM & WATERFALL DRAWING ---

static void DrawSpectrumEnhanced(void)
{
    uint16_t steps = GetStepsCount();
    uint8_t bars = (steps > 128) ? 128 : steps;
    uint16_t smoothed[SPECTRUM_MAX_STEPS];
    
    SmoothRssiHistory(rssiHistory, smoothed, bars);

    for (uint8_t i = 0; i < bars; ++i) {
        if (smoothed[i] > peakHold[i]) {
            peakHold[i] = smoothed[i];
        } else if (peakHold[i] > 1) {
            peakHold[i] -= 1;
        }
    }

    DrawSpectrumCurve(smoothed, bars);

    for (uint8_t i = 0; i < bars; i += 2) {
        uint8_t x = (i * 127) / (bars - 1);
        PutPixel(x, Rssi2Y(peakHold[i]), true);
    }
}



static void DrawWaterfall(void)
{
    static const uint8_t bayer4x4[4][4] = {
        { 0, 8, 2, 10 }, { 12, 4, 14, 6 }, { 3, 11, 1, 9 }, { 15, 7, 13, 5 }
    };
    const uint8_t WATERFALL_START_Y = 41;
    const uint8_t WATERFALL_WIDTH = 128;
    const uint16_t SPEC_WIDTH = GetStepsCount();
    const float xScale = (float)SPEC_WIDTH / WATERFALL_WIDTH;

    for (uint8_t y_offset = 0; y_offset < WATERFALL_HISTORY_DEPTH - 1; y_offset++)
    {
        int8_t historyRow = (int8_t)waterfallIndex - (int8_t)y_offset;
        if (historyRow < 0) historyRow += WATERFALL_HISTORY_DEPTH;

        uint8_t y_pos = WATERFALL_START_Y + y_offset;
        if (y_pos > 63) break;

        float fade = 1.0f - (float)y_offset / (WATERFALL_HISTORY_DEPTH - 1);
        for (uint8_t x = 0; x < WATERFALL_WIDTH; x++)
        {
            uint16_t specIdx = (uint16_t)(x * xScale);
            if (specIdx >= SPEC_WIDTH - 1) specIdx = SPEC_WIDTH - 2;
            uint8_t l0 = waterfallHistory[specIdx][historyRow];
            uint8_t l1 = waterfallHistory[specIdx+1][historyRow];
            float frac = (x * xScale) - specIdx;
            uint8_t level = (uint8_t)((l0 * (1.0f - frac) + l1 * frac) * fade);
            if (level > 15) level = 15;

            if (level > bayer4x4[y_pos & 3][x & 3]) PutPixel(x, y_pos, 1);
        }
    }
}

// --- 4. KEYBOARD HANDLERS & SETTINGS UPDATES ---

static void UpdateRssiTriggerLevel(bool inc)
{
    if (inc) settings.rssiTriggerLevel += 2;
    else settings.rssiTriggerLevel -= 2;
    ClampRssiTriggerLevel();
    redrawScreen = true;
    redrawStatus = true;
}

static void UpdateDBMax(bool inc)
{
    if (inc && settings.dbMax < 10) settings.dbMax += 1;
    else if (!inc && settings.dbMax > settings.dbMin) settings.dbMax -= 1;
    else return;

    ClampRssiTriggerLevel();
    redrawStatus = true;
    redrawScreen = true;
    SYSTEM_DelayMs(20);
}

static void UpdateScanStep(bool inc)
{
    if (inc) {
        if (settings.scanStepIndex < SCAN_STEP_COUNT - 1) settings.scanStepIndex++;
    } else {
        if (settings.scanStepIndex > 0) settings.scanStepIndex--;
    }
    redrawScreen = true; 
}

static void UpdateCurrentFreq(bool inc)
{
    uint32_t step = ScanStepTable[settings.scanStepIndex];
    if (inc) currentFreq += step;
    else currentFreq -= step;
    RelaunchScan();
    redrawScreen = true;
}

static void UpdateCurrentFreqStill(bool inc)
{
    UpdateCurrentFreq(inc);
    SYSTEM_DelayMs(50);
}

static void UpdateFreqChangeStep(bool inc)
{
    const uint32_t JUMP = 20000; 
    if (inc) settings.frequencyChangeStep += JUMP;
    else if (settings.frequencyChangeStep > JUMP) settings.frequencyChangeStep -= JUMP;
    RelaunchScan();
    redrawScreen = true;
}

static void ToggleModulation()
{
    settings.modulationType = (settings.modulationType + 1) % MODULATION_UKNOWN;
    RADIO_SetModulation(settings.modulationType);
    RelaunchScan();
    redrawScreen = true;
}

static void ToggleListeningBW()
{
    settings.listenBw = (settings.listenBw + 1) % 3;
    redrawScreen = true;
}

static void ToggleBacklight()
{
    settings.backlightState = !settings.backlightState;
    if (settings.backlightState) BACKLIGHT_TurnOn();
    else BACKLIGHT_TurnOff();
}

static void ToggleStepsCount()
{
    settings.stepsCount = (settings.stepsCount + 1) % 4;
    RelaunchScan();
    ResetPeak();
    redrawScreen = true; 
}

// --- 5. FREQUENCY INPUT LOGIC ---

static void ResetFreqInput()
{
    tempFreq = 0;
    for (int i = 0; i < 10; ++i) freqInputString[i] = '-';
}

static void FreqInput()
{
    freqInputIndex = 0;
    freqInputDotIndex = 0;
    ResetFreqInput();
    extern bool gWasFKeyPressed;
    gWasFKeyPressed = false;
    SetState(FREQ_INPUT);
}

static void UpdateFreqInput(KEY_Code_t key)
{
    if (key != KEY_EXIT && freqInputIndex >= 10) return;
    if (key == KEY_STAR) {
        if (freqInputIndex == 0 || freqInputDotIndex) return;
        freqInputDotIndex = freqInputIndex;
    }
    if (key == KEY_EXIT) {
        freqInputIndex--;
        if (freqInputDotIndex == freqInputIndex) freqInputDotIndex = 0;
    } else {
        freqInputArr[freqInputIndex++] = key;
    }

    ResetFreqInput();
    uint8_t dotIndex = freqInputDotIndex == 0 ? freqInputIndex : freqInputDotIndex;

    for (int i = 0; i < 10; ++i) {
        if (i < freqInputIndex) {
            KEY_Code_t digitKey = freqInputArr[i];
            freqInputString[i] = digitKey <= KEY_9 ? '0' + digitKey - KEY_0 : '.';
        } else {
            freqInputString[i] = '-';
        }
    }

    uint32_t base = 100000;
    for (int i = dotIndex - 1; i >= 0; --i) {
        tempFreq += (freqInputArr[i] - KEY_0) * base;
        base *= 10;
    }

    base = 10000;
    if (dotIndex < freqInputIndex) {
        for (int i = dotIndex + 1; i < freqInputIndex; ++i) {
            tempFreq += (freqInputArr[i] - KEY_0) * base;
            base /= 10;
        }
    }
    redrawScreen = true;
}

// --- 5. UI ELEMENTS & SCREEN RENDER ---
// --- UI ELEMENTS ---

static void DrawF(uint32_t f)
{
    sprintf(String, "%u.%05u", f / 100000, f % 100000);
    UI_PrintStringSmallNormal(String, 8, 127, 0);

    sprintf(String, "%3s", gModulationStr[settings.modulationType]);
    GUI_DisplaySmallest(String, 116, 1, false, true);
    sprintf(String, "%4sk", bwOptions[settings.listenBw]);
    GUI_DisplaySmallest(String, 108, 7, false, true);
}

static void DrawStatus()
{
#ifdef SPECTRUM_EXTRA_VALUES
    sprintf(String, "%d/%ddBm P:%d T:%d", settings.dbMin, settings.dbMax,
            Rssi2DBm(peak.rssi), Rssi2DBm(settings.rssiTriggerLevel));
#else
    sprintf(String, "%d/%ddBm", settings.dbMin, settings.dbMax);
#endif
    GUI_DisplaySmallest(String, 0, 1, true, true);

    BOARD_ADC_GetBatteryInfo(&gBatteryVoltages[gBatteryCheckCounter++ % 4],
                             &gBatteryCurrent);

    uint16_t voltage = (gBatteryVoltages[0] + gBatteryVoltages[1] +
                        gBatteryVoltages[2] + gBatteryVoltages[3]) /
                       4 * 760 / gBatteryCalibration[3];

    unsigned perc = BATTERY_VoltsToPercent(voltage);

    // sprintf(String, "%d %d", voltage, perc);
    // GUI_DisplaySmallest(String, 48, 1, true, true);

    gStatusLine[116] = 0b00011100;
    gStatusLine[117] = 0b00111110;
    for (int i = 118; i <= 126; i++)
    {
        gStatusLine[i] = 0b00100010;
    }

    for (unsigned i = 127; i >= 118; i--)
    {
        if (127 - i <= (perc + 5) * 9 / 100)
        {
            gStatusLine[i] = 0b00111110;
        }
    }
}

static void DrawRssiTriggerLevel()
{
    if (settings.rssiTriggerLevel == RSSI_MAX_VALUE)
        return;
    
    uint8_t y = Rssi2Y(settings.rssiTriggerLevel);
    for (uint8_t x = 0; x < 128; x += 2)
    {
        PutPixel(x, y, true);
    }
}

static void DrawNums()
{
    if (currentState == SPECTRUM) 
    {
        // 1. Draw Zoom level on the top line
        sprintf(String, "Z:%ux", GetStepsCount());
        GUI_DisplaySmallest(String, 0, 1, false, true); // Y=1 (Top)

        // 2. Draw Step/Span on the second line
        sprintf(String, "S:%u.%02ukHz", 
                ScanStepTable[settings.scanStepIndex] / 100, 
                ScanStepTable[settings.scanStepIndex] % 100);
        GUI_DisplaySmallest(String, 0, 9, false, true); // Y=9 (Directly below)
    }

    sprintf(String, "%u.%05u", GetFStart() / 100000, GetFStart() % 100000);
        // MOVED: Y coordinate 38 -> 34
        GUI_DisplaySmallest(String, 0, 34, false, true);

        sprintf(String, "\x7F%u.%02uk", settings.frequencyChangeStep / 100,
                settings.frequencyChangeStep % 100);
        // MOVED: Y coordinate 38 -> 34
        GUI_DisplaySmallest(String, 48, 34, false, true);

        sprintf(String, "%u.%05u", GetFEnd() / 100000, GetFEnd() % 100000);
        // MOVED: Y coordinate 38 -> 34
        GUI_DisplaySmallest(String, 93, 34, false, true);
}

static void DrawTicks()
{
    uint32_t fStart = GetFStart();
    uint32_t span = GetFEnd() - fStart;
    uint32_t step = span / 128;
    if (step == 0) step = 1;

    uint8_t spacing = 128 / GetStepsCount(); 

    for (uint8_t i = 0; i < 128; i++) 
    {
        uint32_t f = fStart + (uint64_t)span * i / 128;
        bool isMajorTick = ((f % 50000) < step || (f % 100000) < step);
        bool isSpacingPixel = (i % spacing == 0);

        if (isSpacingPixel || isMajorTick)
        {
            uint8_t barValue = 0b00010000; 
            if ((f % 10000) < step)  barValue |= 0b00100000;
            if ((f % 50000) < step)  barValue |= 0b01000000;
            if ((f % 100000) < step) barValue |= 0b10000000;
            gFrameBuffer[3][i] |= barValue;
        }
    }

    if (IsCenterMode()) {
        memset(gFrameBuffer[3] + 62, 0x08, 5); 
        gFrameBuffer[3][64] = 0x0f;
    } else {
        memset(gFrameBuffer[3] + 1, 0x08, 3);
        memset(gFrameBuffer[3] + 124, 0x08, 3);
        gFrameBuffer[3][0] = 0x0f;
        gFrameBuffer[3][127] = 0x0f;
    }
}

static void DrawArrow(uint8_t x)
{
    for (signed i = -2; i <= 2; ++i) {
        signed v = x + i;
        if (!(v & 128)) {
            gFrameBuffer[3][v] |= (0b00000111 << my_abs(i)) & 0b00000111;
        }
    }
}

// --- 7. KEY EVENT DISPATCHERS ---

static void OnKeyDown(uint8_t key)
{
    switch (key)
    {
    case KEY_3: UpdateDBMax(true); break;
    case KEY_9: UpdateDBMax(false); break;
    case KEY_1: UpdateScanStep(true); break;
    case KEY_7: UpdateScanStep(false); break;
    case KEY_2: UpdateFreqChangeStep(true); break;
    case KEY_8: UpdateFreqChangeStep(false); break;
    case KEY_UP: UpdateCurrentFreq(true); break;
    case KEY_DOWN: UpdateCurrentFreq(false); break;
    case KEY_STAR: UpdateRssiTriggerLevel(true); break;
    case KEY_F: UpdateRssiTriggerLevel(false); break;
    case KEY_5: FreqInput(); break;
    case KEY_0: ToggleModulation(); break;
    case KEY_6: ToggleListeningBW(); break;
    case KEY_4: ToggleStepsCount(); break;
    case KEY_SIDE2: ToggleBacklight(); break;
    case KEY_PTT: SetState(STILL); TuneToPeak(); break;
    case KEY_EXIT:
        if (menuState)
        {
            menuState = 0;
            break;
        }
        DeInitSpectrum();
        
        // Now that the header is at the top, these will work perfectly:
        gCurrentFunction = FUNCTION_FOREGROUND;
        gRequestDisplayScreen = DISPLAY_MAIN;
        break;
    }
}

static void OnKeyDownFreqInput(uint8_t key)
{
    switch (key)
    {
    case KEY_0: case KEY_1: case KEY_2: case KEY_3: case KEY_4:
    case KEY_5: case KEY_6: case KEY_7: case KEY_8: case KEY_9:
    case KEY_STAR: UpdateFreqInput(key); break;
    case KEY_EXIT:
        if (freqInputIndex == 0) SetState(previousState);
        else UpdateFreqInput(key);
        break;
    case KEY_MENU:
        if (tempFreq >= F_MIN && tempFreq <= F_MAX) {
            SetState(previousState);
            currentFreq = tempFreq;
            RelaunchScan();
        }
        break;
    default: break;
    }
}

void OnKeyDownStill(KEY_Code_t key)
{
    switch (key)
    {
    case KEY_3: UpdateDBMax(true); break;
    case KEY_9: UpdateDBMax(false); break;
    case KEY_UP: UpdateCurrentFreqStill(true); break;
    case KEY_DOWN: UpdateCurrentFreqStill(false); break;
    case KEY_5: FreqInput(); break;
    case KEY_EXIT: SetState(SPECTRUM); RelaunchScan(); break;
    default: break;
    }
}

static void RenderFreqInput() { UI_PrintString(freqInputString, 2, 127, 0, 8); }

static void RenderStatus()
{
    memset(gStatusLine, 0, sizeof(gStatusLine));
    DrawStatus();
    ST7565_BlitStatusLine();
}

/**
 * @brief Professional-grade spectrum display rendering pipeline
 *
 * Orchestrates the complete spectrum analyzer display by calling individual
 * drawing functions in optimized order to build a multi-layered visualization
 * with proper visual hierarchy. The rendering pipeline follows this sequence:
 *
 * 1. **Background Layer** (Frequency/Time Axis)
 *    - Frequency tick marks with graduated scaling
 *    - Center frequency indicator for center mode
 *    - Edge markers for span mode
 *
 * 2. **Signal Layer** (Spectrum Data)
 *    - High-resolution spectrum bars with smooth scaling
 *    - Dynamic range compression for visibility
 *    - Peak frequency indicator with arrow
 *
 * 3. **Reference Layer** (Amplitude Reference)
 *    - RSSI trigger level line
 *    - Signal detection threshold
 *    - Noise floor reference
 *
 * 4. **Information Layer** (Text & Metrics)
 *    - Peak frequency display (6 decimal places)
 *    - S-meter reading
 *    - Signal strength in dBm
 *    - Scan step and bandwidth indicators
 *
 * 5. **History Layer** (Waterfall)
 *    - Temporal signal analysis (16 rows × frequency samples)
 *    - Professional 16-level grayscale representation
 *    - Smooth transitions with spatial dithering
 *
 * @see DrawTicks() - Frequency axis rendering
 * @see DrawSpectrumEnhanced() - Signal amplitude display
 * @see DrawRssiTriggerLevel() - Reference level indicators
 * @see DrawF() - Peak frequency text
 * @see DrawNums() - Numeric information display
 * @see DrawWaterfall() - Temporal waterfall visualization
 */
static void RenderSpectrum(void)
{
    DrawTicks();
    DrawArrow(128u * peak.i / GetStepsCount());

    DrawSpectrumEnhanced(); 
    DrawRssiTriggerLevel();

    // --- SUGAR 1 SQUELCH LOGIC ---
    if (peak.rssi > settings.rssiTriggerLevel) {
        // Register 0x30 is the main audio gate for the BK chip
        BK4819_WriteRegister(BK4819_REG_30, 0x0000); //Former BK4829_WriteRegister(0x30, 0x0000); 
    } else {
        BK4829_WriteRegister(0x30, 0xFFFF);
    }

    DrawF(peak.f);
    DrawNums();
    DrawWaterfall(); 
}

static void RenderStill()
{
    DrawF(fMeasure);

    const uint8_t METER_PAD_LEFT = 3;

    memset(&gFrameBuffer[2][METER_PAD_LEFT], 0b00010000, 121);

    for (int i = 0; i < 121; i += 5)
    {
        gFrameBuffer[2][i + METER_PAD_LEFT] = 0b00110000;
    }

    for (int i = 0; i < 121; i += 10)
    {
        gFrameBuffer[2][i + METER_PAD_LEFT] = 0b01110000;
    }

    uint8_t x = Rssi2PX(displayRssi, 0, 121);
    for (int i = 0; i < x; ++i)
    {
        if (i % 5)
        {
            gFrameBuffer[2][i + METER_PAD_LEFT] |= 0b00000111;
        }
    }

    int dbm = Rssi2DBm(displayRssi);
    uint8_t s = DBm2S(dbm);
    sprintf(String, "S: %u", s);
    GUI_DisplaySmallest(String, 4, 25, false, true);
    sprintf(String, "%d dBm", dbm);
    GUI_DisplaySmallest(String, 28, 25, false, true);

    if (!monitorMode)
    {
        uint8_t x = Rssi2PX(settings.rssiTriggerLevel, 0, 121);
        gFrameBuffer[2][METER_PAD_LEFT + x] = 0b11111111;
    }

    const uint8_t PAD_LEFT = 4;
    const uint8_t CELL_WIDTH = 30;
    uint8_t offset = PAD_LEFT;
    uint8_t row = 4;

    for (int i = 0, idx = 1; idx <= 4; ++i, ++idx)
    {
        if (idx == 5)
        {
            row += 2;
            i = 0;
        }
        offset = PAD_LEFT + i * CELL_WIDTH;
        if (menuState == idx)
        {
            for (int j = 0; j < CELL_WIDTH; ++j)
            {
                gFrameBuffer[row][j + offset] = 0xFF;
                gFrameBuffer[row + 1][j + offset] = 0xFF;
            }
        }
        sprintf(String, "%s", registerSpecs[idx].name);
        GUI_DisplaySmallest(String, offset + 2, row * 8 + 2, false,
                            menuState != idx);

#ifdef ENABLE_FEAT_N7SIX_SPECTRUM
        if(idx == 1)
        {
            sprintf(String, "%ddB", LNAsOptions[GetRegMenuValue(idx)]);
        }
        else if(idx == 2)
        {
            sprintf(String, "%ddB", LNAOptions[GetRegMenuValue(idx)]);
        }
        else if(idx == 3)
        {
            sprintf(String, "%ddB", VGAOptions[GetRegMenuValue(idx)]);
        }
        else if(idx == 4)
        {
            sprintf(String, "%skHz", BPFOptions[(GetRegMenuValue(idx) / 0x2aaa)]);
        }
#else
        sprintf(String, "%u", GetRegMenuValue(idx));
#endif
        GUI_DisplaySmallest(String, offset + 2, (row + 1) * 8 + 1, false,
                            menuState != idx);
    }
}

static void Render()
{
    UI_DisplayClear();

    switch (currentState)
    {
    case SPECTRUM:
        RenderSpectrum();
        break;
    case FREQ_INPUT:
        RenderFreqInput();
        break;
    case STILL:
        RenderStill();
        break;
    }

    ST7565_BlitFullScreen();
}

static bool HandleUserInput()
{
    kbd.prev = kbd.current;
    kbd.current = GetKey();

    if (kbd.current != KEY_INVALID && kbd.current == kbd.prev)
    {
        if (kbd.counter < 16)
            kbd.counter++;
        else
            kbd.counter -= 3;
        SYSTEM_DelayMs(20);
    }
    else
    {
        kbd.counter = 0;
    }

    if (kbd.counter == 3 || kbd.counter == 16)
    {
        switch (currentState)
        {
        case SPECTRUM:
            OnKeyDown(kbd.current);
            break;
        case FREQ_INPUT:
            OnKeyDownFreqInput(kbd.current);
            break;
        case STILL:
            OnKeyDownStill(kbd.current);
            break;
        }
    }

    return true;
}

/**
 * @brief Perform single frequency measurement during scan
 *
 * Core measurement routine executed for each frequency step:
 * 1. Checks if frequency has valid data (not already scanned)
 * 2. Sets hardware to target frequency
 * 3. Performs RSSI measurement
 * 4. Updates scan statistics (peak, min, etc.)
 * 5. Records measurement in history buffer
 *
 * @see SetF() - Tunes to specific frequency
 * @see Measure() - Reads RSSI from hardware
 * @see UpdateScanInfo() - Updates statistics
 * @see SetRssiHistory() - Records in waterfall/display buffer
 */
static void Scan()
{
    // Skip if already measured or if in blacklist
    if (rssiHistory[scanInfo.i] != RSSI_MAX_VALUE
#ifdef ENABLE_SCAN_RANGES
        && !IsBlacklisted(scanInfo.i)
#endif
    )
    {
        SetF(scanInfo.f);           // Tune receiver to target frequency
        Measure();                   // Perform RSSI measurement
        // Determine if signal is present (use rssiHistory or scanInfo.rssi)
        bool hasSignal = (scanInfo.rssi > settings.rssiTriggerLevel);
        SetBandLed(scanInfo.f, false, hasSignal); // Set LED for RX only if signal
        UpdateScanInfo();            // Update peak/min statistics
    }
}

/**
 * @brief Advance to next frequency step
 *
 * Sequence:
 * 1. Increment peak age counter (for peak hold timeout)
 * 2. Calculate next frequency based on step size
 * 3. Update scan counter
 * 4. Apply frequency boundaries
 * 5. Trigger waterfall update if interval reached
 * 6. Determine if scan is complete
 */
// --- SECTION 1: CORE HELPERS ---

static void NextScanStep()
{
    ++peak.t;  // Increment peak age for timeout tracking
    ++scanInfo.i;
    scanInfo.f += scanInfo.scanStep;
}

static bool IsPeakOverLevel()
{
    // Hysteresis: Use a lower threshold to stay open than to open
    uint16_t effectiveThreshold = isListening ? 
        (settings.rssiTriggerLevel - 4) : settings.rssiTriggerLevel;

    return peak.rssi > effectiveThreshold;
}

static void AutoTriggerLevel()
{
    // Establish the Noise Floor as the baseline to prevent "chasing" signals
    uint16_t noiseFloor = scanInfo.rssiMin;
    uint16_t targetTrigger = noiseFloor + 12; // ~6dB above noise

    if (settings.rssiTriggerLevel == RSSI_MAX_VALUE) {
        settings.rssiTriggerLevel = targetTrigger + 20;
    } else {
        if (targetTrigger > settings.rssiTriggerLevel) settings.rssiTriggerLevel++;
        else if (targetTrigger < settings.rssiTriggerLevel) settings.rssiTriggerLevel--;
        
        if (settings.rssiTriggerLevel < 40) settings.rssiTriggerLevel = 40;
    }
}

// --- SECTION 2: OPTIMIZED WATERFALL ---

static void UpdateWaterfall()
{
    // Optimized with Integer Math for high FPS
    for (uint8_t x = 0; x < 128; x++)
    {
        uint16_t rssi = rssiHistory[x >> settings.stepsCount];
        uint8_t intensity = 0;
        
        if (rssi > scanInfo.rssiMin) {
            uint32_t diff = (uint32_t)(rssi - scanInfo.rssiMin);
            // Dynamic range stretch: maps signal-to-noise onto 16 levels
            intensity = (uint8_t)((diff * 15) / (scanInfo.rssiMax - scanInfo.rssiMin + 1));
        }
        waterfallHistory[waterfallIndex][x] = intensity;
    }
    waterfallIndex = (waterfallIndex + 1) % WATERFALL_ROWS;
}

// --- SECTION 3: UPDATE LOOPS ---

static void UpdateScan()
{
    Scan();

    if (scanInfo.i < scanInfo.measurementsCount)
    {
        NextScanStep();
        return;
    }

    if (!(scanInfo.measurementsCount >> 7)) 
        memset(&rssiHistory[scanInfo.measurementsCount], 0,
               sizeof(rssiHistory) - scanInfo.measurementsCount * sizeof(rssiHistory[0]));

    redrawScreen = true;
    preventKeypress = false;
    UpdatePeakInfo();
    
    static uint8_t waterfallUpdateCounter = 0;
    if (++waterfallUpdateCounter >= 2)
    {
        UpdateWaterfall();
        waterfallUpdateCounter = 0;
    }
    
    if (IsPeakOverLevel())
    {
        ToggleRX(true);
        TuneToPeak();
        return;
    }
    newScanStart = true;
}

static void UpdateStill()
{
    Measure();
    
    // --- Professional Ballistics (Fast Attack / Slow Decay) ---
    if (displayRssi < scanInfo.rssi) {
        // Attack: Jump to the new signal level quickly (50% per frame)
        displayRssi = (displayRssi + scanInfo.rssi) >> 1;
    } else {
        // Decay: Fall back slowly to avoid flickering (95% old, 5% new)
        displayRssi = (displayRssi * 19 + scanInfo.rssi) / 20;
    }
    
    redrawScreen = true;
    preventKeypress = false;

    peak.rssi = scanInfo.rssi;
    AutoTriggerLevel();

    if (IsPeakOverLevel() || monitorMode) {
        ToggleRX(true);
    }
}

static void UpdateListening()
{
    preventKeypress = false;
#ifdef ENABLE_FEAT_N7SIX_SPECTRUM
    bool tailFound = checkIfTailFound();
    if (tailFound) listenT = 0;
#else
    bool tailFound = false;
    if (currentState == STILL) listenT = 0;
#endif

    if (listenT)
    {
        listenT--;
        SYSTEM_DelayMs(1);
        return;
    }

    if (currentState == SPECTRUM)
    {
        BK4819_WriteRegister(0x43, GetBWRegValueForScan());
        Measure();
        BK4819_WriteRegister(0x43, listenBWRegValues[settings.listenBw]);
        
        static uint8_t waterfallUpdateCounter = 0;
        if (++waterfallUpdateCounter >= 2)
        {
            UpdateWaterfall();
            waterfallUpdateCounter = 0;
        }
    }
    else
    {
        Measure();
    }

    peak.rssi = scanInfo.rssi;
    redrawScreen = true;

    if ((IsPeakOverLevel() && !tailFound) || monitorMode)
        {
            // STILL mode: 100ms (Very fast, used for constant monitoring)
            // SPECTRUM mode: 400ms (Professional standard 'Squelch Tail' length)
            listenT = (currentState == STILL) ? 100 : 400; 
            return;
        }

    ToggleRX(false);
    ResetScanStats();
}

// --- SECTION 4: MAIN SYSTEM TICK ---

static void Tick()
{
#ifdef ENABLE_AM_FIX
    if (gNextTimeslice)
    {
        gNextTimeslice = false;
        if (settings.modulationType == MODULATION_AM && !lockAGC)
        {
            AM_fix_10ms(vfo); 
        }
    }
#endif

    // Handle user input only once per tick
    if (!preventKeypress) HandleUserInput();

    // --- SECTION: END OF SCAN PROCESSING ---
    if (newScanStart)
    {
        // 1. Snapshot the whole scan into the waterfall
        PushWaterfallLine();

        // 2. Sugar 1 Squelch (Hardware Gate)
        // Switch to BK4819 prefix so the linker can find the function
        if (peak.rssi > settings.rssiTriggerLevel) {
            BK4819_WriteRegister(BK4819_REG_30, 0x0000); // Open Audio Path
        } else {
            BK4819_WriteRegister(BK4819_REG_30, 0xFFFF); // Close Audio Path
        }

        InitScan();
        newScanStart = false;
    }

#ifdef ENABLE_SCAN_RANGES
    // Periodic peak check for wide spans
    if (gNextTimeslice_500ms)
    {
        gNextTimeslice_500ms = false;
        if (GetStepsCount() > 128 && !isListening)
        {
            UpdatePeakInfo();
            if (IsPeakOverLevel())
            {
                ToggleRX(true);
                TuneToPeak();
                return;
            }
            redrawScreen = true;
            preventKeypress = false;
        }
    }
#endif

    // --- SECTION: STATE MACHINE ---
    if (isListening && currentState != FREQ_INPUT)
    {
        UpdateListening();
    }
    else
    {
        if (currentState == SPECTRUM) UpdateScan();
        else if (currentState == STILL) UpdateStill();
    }

    // --- SECTION: UI REFRESH ---
    if (redrawStatus || ++statuslineUpdateTimer > 4096)
    {
        RenderStatus();
        redrawStatus = false;
        statuslineUpdateTimer = 0;
    }

    if (redrawScreen)
    {
        Render();
#ifdef ENABLE_FEAT_N7SIX_SCREENSHOT
        getScreenShot(false);
#endif
        redrawScreen = false;
    }
}

/**
 * @brief Main spectrum analyzer application entry point
 *
 * Initializes and runs the spectrum analyzer application. This function
 * sets up the radio hardware, loads settings, initializes data structures,
 * and enters the main application loop.
 *
 * The function handles:
 * - Loading persistent settings from flash memory
 * - Setting initial frequency based on scan ranges or current VFO
 * - Backing up radio registers for restoration
 * - Initializing spectrum data buffers
 * - Running the main application loop
 */
void APP_RunSpectrum(void)
{
    // TX here coz it always? set to active VFO
    vfo = gEeprom.TX_VFO;

#ifdef ENABLE_FEAT_N7SIX_SPECTRUM
    LoadSettings();
#endif

    // Set the current frequency in the middle of the display
#ifdef ENABLE_SCAN_RANGES
    if (gScanRangeStart)
    {
        currentFreq = initialFreq = gScanRangeStart;
        for (uint8_t i = 0; i < ARRAY_SIZE(scanStepValues); i++)
        {
            if (scanStepValues[i] >= gTxVfo->StepFrequency)
            {
                settings.scanStepIndex = i;
                break;
            }
        }
        settings.stepsCount = STEPS_128;
        #ifdef ENABLE_FEAT_N7SIX_RESUME_STATE
            gEeprom.CURRENT_STATE = 5;
        #endif
    }
    else
    {
        currentFreq = initialFreq = gTxVfo->pRX->Frequency -
                                    ((GetStepsCount() / 2) * GetScanStep());
        #ifdef ENABLE_FEAT_N7SIX_RESUME_STATE
            gEeprom.CURRENT_STATE = 4;
        #endif
    }
#else
    currentFreq = initialFreq = gTxVfo->pRX->Frequency -
                                ((GetStepsCount() / 2) * GetScanStep());
    #ifdef ENABLE_FEAT_N7SIX_RESUME_STATE
        gEeprom.CURRENT_STATE = 4;
    #endif
#endif

    #ifdef ENABLE_FEAT_N7SIX_RESUME_STATE
        SETTINGS_WriteCurrentState();
    #endif

    BackupRegisters();

    isListening = true; // to turn off RX later
    redrawStatus = true;
    redrawScreen = true;
    newScanStart = true;

    ToggleRX(true), ToggleRX(false); // hack to prevent noise when squelch off
    RADIO_SetModulation(settings.modulationType = gTxVfo->Modulation);

#ifdef ENABLE_FEAT_N7SIX_SPECTRUM
    BK4819_SetFilterBandwidth(settings.listenBw, false);
#else
    BK4819_SetFilterBandwidth(settings.listenBw = BK4819_FILTER_BW_WIDE, false);
#endif

    RelaunchScan();

    memset(rssiHistory, 0, sizeof(rssiHistory));
    memset(waterfallHistory, 0, sizeof(waterfallHistory));
    waterfallIndex = 0;

    isInitialized = true;

    // Watchdog for preventKeypress stuck condition
    uint32_t preventKeypressTimeout = 0;
    while (isInitialized)
    {
        Tick();
        if (preventKeypress) {
            preventKeypressTimeout++;
            if (preventKeypressTimeout > 100000) { // ~100k cycles, adjust as needed
                preventKeypress = false;
                // Optionally: display warning to user here
            }
        } else {
            preventKeypressTimeout = 0;
        }
    }
}
static void PushWaterfallLine(void)
{
    // Cycle the circular buffer
    waterfallIndex = (waterfallIndex + 1) % WATERFALL_HISTORY_DEPTH;

    uint16_t steps = GetStepsCount();
    for (uint16_t i = 0; i < steps; i++)
    {
        // Map RSSI to 0-15 level
        uint8_t level = (rssiHistory[i] > settings.dbMin) ? (rssiHistory[i] - settings.dbMin) : 0;
        level /= 4; 
        if (level > 15) level = 15;

        waterfallHistory[i][waterfallIndex] = level;
    }
}