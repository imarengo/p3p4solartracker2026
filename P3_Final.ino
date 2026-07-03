#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <Wire.h>
#include <Preferences.h>
#include <driver/gpio.h>

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ============================================================
// WIFI, OTA Y TELNET
// ============================================================

const char* WIFI_SSID = "iPhone de Macri";
const char* WIFI_PASSWORD = "omatopopih";
const char* OTA_HOSTNAME = "ESP32S3_P3P4_G20";
const char* OTA_PASSWORD = "17127674";

WiFiServer telnetServer(23);
WiFiClient telnetClient;

// ============================================================
// PINES
// ============================================================

// Motor 1: azimut
constexpr uint8_t ENCODER_1_A_PIN = 47;
constexpr uint8_t ENCODER_1_B_PIN = 48;
constexpr uint8_t MOTOR_1_PWM_PIN = 1;
constexpr uint8_t MOTOR_1_DIR_PIN = 2;
constexpr uint8_t LIMIT_MOTOR_1_PIN = 17;

// Motor 2: elevacion
constexpr uint8_t ENCODER_2_A_PIN = 14;
constexpr uint8_t ENCODER_2_B_PIN = 21;
constexpr uint8_t MOTOR_2_PWM_PIN = 42;
constexpr uint8_t MOTOR_2_DIR_PIN = 41;
constexpr uint8_t LIMIT_MOTOR_2_PIN = 18;

// RTC RV-3028-C7
constexpr uint8_t RTC_SCL_PIN = 8;
constexpr uint8_t RTC_SDA_PIN = 9;
constexpr uint8_t RTC_CLKOUT_PIN = 10;
constexpr uint8_t RTC_EVI_PIN = 11;
constexpr uint8_t RTC_INT_PIN = 12;

// ============================================================
// MECANICA Y COORDENADAS
// ============================================================

constexpr uint8_t AXIS_COUNT = 2;
constexpr uint8_t AZIMUTH_AXIS = 0;
constexpr uint8_t ELEVATION_AXIS = 1;

const uint8_t MOTOR_PWM_PINS[AXIS_COUNT] = {
    MOTOR_1_PWM_PIN,
    MOTOR_2_PWM_PIN
};

const uint8_t MOTOR_DIR_PINS[AXIS_COUNT] = {
    MOTOR_1_DIR_PIN,
    MOTOR_2_DIR_PIN
};

const uint8_t LIMIT_PINS[AXIS_COUNT] = {
    LIMIT_MOTOR_1_PIN,
    LIMIT_MOTOR_2_PIN
};

// Cambiar individualmente si un motor gira al reves.
const uint8_t MOTOR_POSITIVE_DIR_LEVEL[AXIS_COUNT] = {
    LOW,
    LOW
};

// Cambiar individualmente si un encoder cuenta al reves.
constexpr int8_t ENCODER_1_SIGN = 1;
constexpr int8_t ENCODER_2_SIGN = 1;

constexpr double ENCODER_BASIC_PPR = 16.0;
constexpr double QUADRATURE_FACTOR = 4.0;
constexpr double INTERNAL_REDUCTION = 169.0;
constexpr double EXTERNAL_REDUCTION = 50.0;

constexpr double ANTENNA_COUNTS_PER_REVOLUTION =
    ENCODER_BASIC_PPR *
    QUADRATURE_FACTOR *
    INTERNAL_REDUCTION *
    EXTERNAL_REDUCTION;

constexpr double AZIMUTH_MIN_DEGREES = 0.0;
constexpr double AZIMUTH_MAX_DEGREES = 360.0;

constexpr double ELEVATION_MIN_DEGREES = 0.0;
constexpr double ELEVATION_MAX_DEGREES = 90.0;

// Referencia fisica obtenida con los finales de carrera.
constexpr double HOME_AZIMUTH_DEGREES = 0.0;
constexpr double HOME_ELEVATION_DEGREES = 90.0;

// Posicion nocturna.
constexpr double PARK_AZIMUTH_DEGREES = 0.0;
constexpr double PARK_ELEVATION_DEGREES = 0.0;

/*
 * Direccion de coordenada hacia cada final:
 *
 * Azimut:
 * hacia 0 grados => -1
 *
 * Elevacion:
 * hacia 90 grados => +1
 *
 * Si fisicamente algun eje se aleja de su final durante
 * el homing, cambie solamente el signo correspondiente.
 */
constexpr int8_t HOMING_TOWARD_LIMIT_DIRECTION[AXIS_COUNT] = {
    -1,
    +1
};

// ============================================================
// PWM Y MOVIMIENTO
// ============================================================

constexpr uint32_t PWM_FREQUENCY_HZ = 20000;
constexpr uint8_t PWM_RESOLUTION_BITS = 8;
constexpr uint8_t PWM_MAX = 255;

constexpr uint8_t MIN_POSITION_PWM = 55;

constexpr uint8_t SLEW_MAX_PWM = 180;
constexpr uint8_t PARK_MAX_PWM = 170;
constexpr uint8_t TRACK_MAX_PWM = 80;
constexpr uint8_t MANUAL_MAX_PWM = 100;

constexpr uint32_t PWM_RAMP_INTERVAL_MS = 20;
constexpr uint8_t PWM_RAMP_UP_STEP = 3;
constexpr uint8_t PWM_RAMP_DOWN_STEP = 5;

constexpr double SLEW_SLOWDOWN_DEGREES = 5.0;
constexpr double PARK_SLOWDOWN_DEGREES = 5.0;
constexpr double TRACK_SLOWDOWN_DEGREES = 1.0;
constexpr double MANUAL_SLOWDOWN_DEGREES = 3.0;

constexpr double POSITION_TOLERANCE_DEGREES = 0.05;
constexpr double SOFT_LIMIT_MARGIN_DEGREES = 0.50;
constexpr double LIMIT_HOME_ACCEPTANCE_DEGREES = 5.0;

constexpr int64_t MIN_ENCODER_ACTIVITY_COUNTS = 8;

constexpr uint32_t MOTOR_STARTUP_GRACE_MS = 700;
constexpr uint32_t ENCODER_STALL_TIMEOUT_MS = 2000;

// ============================================================
// HOMING
// ============================================================

constexpr uint32_t LIMIT_DEBOUNCE_MS = 30;

constexpr uint8_t HOMING_FAST_PWM = 95;
constexpr uint8_t HOMING_SLOW_PWM = 60;
constexpr uint8_t HOMING_RELEASE_PWM = 60;

constexpr double HOMING_BACKOFF_DEGREES = 1.0;
constexpr double HOMING_RELEASE_MAX_DEGREES = 5.0;

constexpr uint32_t HOMING_RELEASE_TIMEOUT_MS = 8000;
constexpr uint32_t HOMING_SWITCH_SETTLE_MS = 150;

constexpr uint32_t HOMING_AZIMUTH_TIMEOUT_MS = 240000;
constexpr uint32_t HOMING_ELEVATION_TIMEOUT_MS = 90000;

constexpr double HOMING_AZIMUTH_MAX_TRAVEL_DEGREES = 370.0;
constexpr double HOMING_ELEVATION_MAX_TRAVEL_DEGREES = 100.0;

// ============================================================
// SEGUIMIENTO SOLAR
// ============================================================

constexpr uint32_t SOLAR_UPDATE_INTERVAL_MS = 30000;

constexpr double TRACKING_DEADBAND_DEGREES = 0.25;
constexpr double SLEW_THRESHOLD_DEGREES = 5.0;

constexpr double NIGHT_ENTER_ELEVATION_DEGREES = -0.5;
constexpr double DAY_ENTER_ELEVATION_DEGREES = 0.5;

// ============================================================
// REINICIO AUTOMATICO POR FAULT
// ============================================================

constexpr uint32_t FAULT_RTC_MAGIC = 0x4641554CUL;

constexpr uint32_t FAULT_RESTART_DELAYS_MS[] = {
    5000,
    10000,
    20000,
    60000
};

constexpr size_t FAULT_RESTART_DELAY_COUNT =
    sizeof(FAULT_RESTART_DELAYS_MS) /
    sizeof(FAULT_RESTART_DELAYS_MS[0]);

RTC_DATA_ATTR uint32_t rtcFaultMagic = 0;
RTC_DATA_ATTR uint32_t consecutiveFaultRestarts = 0;
RTC_DATA_ATTR uint8_t rtcLastFaultCode = 0;
RTC_DATA_ATTR char rtcLastFaultMessage[96] = {0};

// ============================================================
// PERSISTENCIA NVS
// ============================================================

constexpr char PREFERENCES_NAMESPACE[] = "solartrk";

constexpr char PERSISTENT_KEY_A[] = "stateA";
constexpr char PERSISTENT_KEY_B[] = "stateB";

constexpr uint32_t PERSISTENT_MAGIC = 0x534F4C52UL;

constexpr uint16_t PERSISTENT_VERSION_V1 = 1;
constexpr uint16_t PERSISTENT_VERSION_V2 = 2;

// ============================================================
// RTC RV-3028-C7
// ============================================================

constexpr uint8_t RV3028_ADDRESS = 0x52;

constexpr uint8_t RTC_REG_SECONDS = 0x00;
constexpr uint8_t RTC_REG_MINUTES = 0x01;
constexpr uint8_t RTC_REG_HOURS = 0x02;
constexpr uint8_t RTC_REG_WEEKDAY = 0x03;
constexpr uint8_t RTC_REG_DATE = 0x04;
constexpr uint8_t RTC_REG_MONTH = 0x05;
constexpr uint8_t RTC_REG_YEAR = 0x06;

constexpr uint8_t RTC_REG_STATUS = 0x0E;
constexpr uint8_t RTC_REG_CONTROL_1 = 0x0F;
constexpr uint8_t RTC_REG_CONTROL_2 = 0x10;

constexpr uint8_t RTC_REG_EE_ADDRESS = 0x25;
constexpr uint8_t RTC_REG_EE_DATA = 0x26;
constexpr uint8_t RTC_REG_EE_COMMAND = 0x27;

constexpr uint8_t RTC_REG_BACKUP_RAM = 0x37;
constexpr uint8_t RTC_EEPROM_BACKUP_ADDRESS = 0x37;

constexpr uint8_t RTC_STATUS_EEBUSY = 0x80;
constexpr uint8_t RTC_STATUS_BSF = 0x20;
constexpr uint8_t RTC_STATUS_PORF = 0x01;

constexpr uint8_t RTC_CONTROL_1_EERD = 0x08;
constexpr uint8_t RTC_CONTROL_2_12_24 = 0x02;

constexpr uint8_t RTC_BACKUP_OFFSET_LSB = 0x80;
constexpr uint8_t RTC_BACKUP_BSIE = 0x40;
constexpr uint8_t RTC_BACKUP_TCE = 0x20;
constexpr uint8_t RTC_BACKUP_FEDE = 0x10;

constexpr uint8_t RTC_BACKUP_BSM_MASK = 0x0C;
constexpr uint8_t RTC_BACKUP_BSM_LSM = 0x0C;

constexpr uint8_t RTC_BACKUP_TCR_MASK = 0x03;

constexpr uint8_t RTC_BACKUP_REQUIRED_MASK =
    RTC_BACKUP_BSIE |
    RTC_BACKUP_TCE |
    RTC_BACKUP_FEDE |
    RTC_BACKUP_BSM_MASK;

constexpr uint8_t RTC_BACKUP_REQUIRED_VALUE =
    RTC_BACKUP_FEDE |
    RTC_BACKUP_BSM_LSM;

constexpr uint8_t RTC_EE_COMMAND_FIRST = 0x00;
constexpr uint8_t RTC_EE_COMMAND_WRITE_ONE = 0x21;
constexpr uint8_t RTC_EE_COMMAND_READ_ONE = 0x22;

// ============================================================
// TIPOS
// ============================================================

enum class AxisMode : uint8_t
{
    IDLE,
    POSITION,
    JOG,
    HOMING
};

enum class MotionProfile : uint8_t
{
    SLEW,
    PARK,
    TRACK,
    MANUAL_GOTO,
    MANUAL_JOG
};

enum class MovementPurpose : uint8_t
{
    NONE,
    AUTO_SUN,
    AUTO_PARK,
    MANUAL_GOTO,
    MANUAL_JOG
};

enum class TrackerState : uint8_t
{
    STARTUP,
    HOMING,
    RTC_INVALID,
    LOCATION_INVALID,
    READY_HOME,
    PARKED,
    SLEWING_TO_SUN,
    TRACKING,
    SLEWING_TO_PARK,
    MANUAL,
    FAULT_RESTART_PENDING,
    OTA_UPDATE
};

enum class HomingPhase : uint8_t
{
    IDLE,
    START_AXIS,
    RELEASE_SWITCH,
    RELEASE_CLEARANCE,
    SEEK_FAST,
    WAIT_FAST_HIT,
    BACKOFF_RELEASE,
    BACKOFF_CLEARANCE,
    WAIT_BEFORE_SLOW,
    SEEK_SLOW,
    AXIS_COMPLETE,
    COMPLETE
};

enum class FaultCode : uint8_t
{
    NONE = 0,

    LIMIT_1_STUCK,
    LIMIT_2_STUCK,

    LIMIT_1_NOT_FOUND,
    LIMIT_2_NOT_FOUND,

    ENCODER_1_STALL,
    ENCODER_2_STALL,

    ENCODER_1_DIRECTION,
    ENCODER_2_DIRECTION,

    SOFTWARE_LIMIT_1,
    SOFTWARE_LIMIT_2,

    HOMING_TIMEOUT,
    NVS_WRITE_ERROR
};

struct AxisState
{
    AxisMode mode = AxisMode::IDLE;
    MotionProfile profile = MotionProfile::SLEW;

    int8_t direction = 0;

    uint8_t currentPwm = 0;
    uint8_t maximumPwm = 0;

    int64_t startCount = 0;
    int64_t targetCount = 0;
    int64_t lastActivityCount = 0;

    double targetDegrees = 0.0;

    uint32_t movementStartMs = 0;
    uint32_t lastActivityMs = 0;
    uint32_t lastRampMs = 0;

    bool limitReleaseAllowed = false;
    int64_t limitReleaseStartCount = 0;
    uint32_t limitReleaseStartMs = 0;
};

struct DebouncedLimit
{
    bool rawActive = false;
    bool stableActive = false;

    uint32_t rawChangedMs = 0;
};

struct RtcDateTime
{
    uint16_t year = 2000;

    uint8_t month = 1;
    uint8_t day = 1;
    uint8_t weekday = 0;

    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
};

struct SolarPosition
{
    double azimuthDegrees = 0.0;
    double geometricElevationDegrees = 0.0;
    double apparentElevationDegrees = 0.0;

    double declinationDegrees = 0.0;
    double equationOfTimeMinutes = 0.0;
    double hourAngleDegrees = 0.0;
};

// Estructura anterior para migrar ubicacion y TRACK.
struct PersistentRecordV1
{
    uint32_t magic;

    uint16_t version;
    uint16_t size;

    uint32_t sequence;

    uint8_t locationValid;
    uint8_t positionValid;
    uint8_t trackEnabled;
    uint8_t flags;

    double latitudeDegrees;
    double longitudeDegrees;
    double altitudeMeters;

    int64_t azimuthCount;
    int64_t elevationCount;

    uint32_t crc32;
};

// Nueva estructura: la posicion ya no se guarda.
struct PersistentRecordV2
{
    uint32_t magic;

    uint16_t version;
    uint16_t size;

    uint32_t sequence;

    uint8_t locationValid;
    uint8_t trackRequested;
    uint8_t rtcBackupEverProven;
    uint8_t reserved;

    double latitudeDegrees;
    double longitudeDegrees;
    double altitudeMeters;

    uint32_t crc32;
};

// ============================================================
// VARIABLES GLOBALES
// ============================================================

volatile int64_t encoderCounts[AXIS_COUNT] = {
    0,
    0
};

volatile uint8_t previousEncoderStates[AXIS_COUNT] = {
    0,
    0
};

portMUX_TYPE encoderMux =
    portMUX_INITIALIZER_UNLOCKED;

static const int8_t DRAM_ATTR QUADRATURE_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

AxisState axes[AXIS_COUNT];
DebouncedLimit limits[AXIS_COUNT];

MovementPurpose activePurpose =
    MovementPurpose::NONE;

MotionProfile activeMotionProfile =
    MotionProfile::SLEW;

TrackerState trackerState =
    TrackerState::STARTUP;

bool positionCalibrated = false;

bool locationValid = false;
bool trackRequested = false;
bool rtcBackupEverProven = false;

double latitudeDegrees = 0.0;
double longitudeDegrees = 0.0;
double altitudeMeters = 0.0;

bool rtcDetected = false;
bool rtcValid = false;

bool rtcBackupConfigured = false;
bool rtcBackupUsedThisBoot = false;
bool rtcBackupEepromReadable = false;
bool rtcBackupRamMatchesEeprom = false;

uint8_t rtcBootStatusRaw = 0;
uint8_t rtcBackupRamValue = 0;
uint8_t rtcBackupEepromValue = 0;

SolarPosition lastSolarPosition;

bool lastSolarPositionValid = false;
bool daylightStateKnown = false;
bool daylightActive = false;

uint32_t lastSolarUpdateMs = 0;
bool forceSolarUpdate = true;

uint32_t persistentSequence = 0;

PersistentRecordV2 lastCommittedRecord = {};

bool lastCommittedRecordValid = false;

HomingPhase homingPhase =
    HomingPhase::IDLE;

uint8_t homingOrderIndex = 0;
uint8_t homingAxis = ELEVATION_AXIS;

int64_t homingPhaseStartCount = 0;
int64_t homingReleaseCount = 0;

uint32_t homingPhaseStartMs = 0;

bool faultActive = false;

FaultCode activeFaultCode =
    FaultCode::NONE;

char activeFaultMessage[128] = {0};

uint32_t faultRestartAtMs = 0;

bool networkServicesStarted = false;
bool telnetWasConnected = false;

constexpr size_t COMMAND_BUFFER_SIZE = 160;

char telnetCommandBuffer[COMMAND_BUFFER_SIZE];
size_t telnetCommandLength = 0;

uint8_t telnetBytesToIgnore = 0;

constexpr double PI_D =
    3.1415926535897932384626433832795;

// ============================================================
// PROTOTIPOS
// ============================================================

void stopAllMotion(bool clearPurpose = true);
void refreshTrackerState();

bool savePersistentConfig(bool forceWrite);

void enterFault(
    FaultCode code,
    const char* format,
    ...
);

void startHoming();
void serviceAutomaticTracking();

void processCommand(char* command);
void showHelp();

// ============================================================
// UTILIDADES
// ============================================================

double degreesToRadians(double degrees)
{
    return degrees * PI_D / 180.0;
}

double radiansToDegrees(double radians)
{
    return radians * 180.0 / PI_D;
}

double clampDouble(
    double value,
    double minimum,
    double maximum
)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}

double normalizeDegrees360(double value)
{
    value = fmod(value, 360.0);

    if (value < 0.0)
    {
        value += 360.0;
    }

    return value;
}

const char* axisName(uint8_t axis)
{
    return axis == AZIMUTH_AXIS
        ? "AZIMUT"
        : "ELEVACION";
}

const char* trackerStateName(TrackerState state)
{
    switch (state)
    {
        case TrackerState::STARTUP:
            return "STARTUP";

        case TrackerState::HOMING:
            return "HOMING";

        case TrackerState::RTC_INVALID:
            return "RTC_INVALID";

        case TrackerState::LOCATION_INVALID:
            return "LOCATION_INVALID";

        case TrackerState::READY_HOME:
            return "READY_HOME";

        case TrackerState::PARKED:
            return "PARKED";

        case TrackerState::SLEWING_TO_SUN:
            return "SLEWING_TO_SUN";

        case TrackerState::TRACKING:
            return "TRACKING";

        case TrackerState::SLEWING_TO_PARK:
            return "SLEWING_TO_PARK";

        case TrackerState::MANUAL:
            return "MANUAL";

        case TrackerState::FAULT_RESTART_PENDING:
            return "FAULT_RESTART_PENDING";

        case TrackerState::OTA_UPDATE:
            return "OTA_UPDATE";

        default:
            return "UNKNOWN";
    }
}

const char* homingPhaseName(HomingPhase phase)
{
    switch (phase)
    {
        case HomingPhase::IDLE:
            return "IDLE";

        case HomingPhase::START_AXIS:
            return "START_AXIS";

        case HomingPhase::RELEASE_SWITCH:
            return "RELEASE_SWITCH";

        case HomingPhase::RELEASE_CLEARANCE:
            return "RELEASE_CLEARANCE";

        case HomingPhase::SEEK_FAST:
            return "SEEK_FAST";

        case HomingPhase::WAIT_FAST_HIT:
            return "WAIT_FAST_HIT";

        case HomingPhase::BACKOFF_RELEASE:
            return "BACKOFF_RELEASE";

        case HomingPhase::BACKOFF_CLEARANCE:
            return "BACKOFF_CLEARANCE";

        case HomingPhase::WAIT_BEFORE_SLOW:
            return "WAIT_BEFORE_SLOW";

        case HomingPhase::SEEK_SLOW:
            return "SEEK_SLOW";

        case HomingPhase::AXIS_COMPLETE:
            return "AXIS_COMPLETE";

        case HomingPhase::COMPLETE:
            return "COMPLETE";

        default:
            return "UNKNOWN";
    }
}

const char* faultCodeName(FaultCode code)
{
    switch (code)
    {
        case FaultCode::NONE:
            return "NONE";

        case FaultCode::LIMIT_1_STUCK:
            return "LIMIT_1_STUCK";

        case FaultCode::LIMIT_2_STUCK:
            return "LIMIT_2_STUCK";

        case FaultCode::LIMIT_1_NOT_FOUND:
            return "LIMIT_1_NOT_FOUND";

        case FaultCode::LIMIT_2_NOT_FOUND:
            return "LIMIT_2_NOT_FOUND";

        case FaultCode::ENCODER_1_STALL:
            return "ENCODER_1_STALL";

        case FaultCode::ENCODER_2_STALL:
            return "ENCODER_2_STALL";

        case FaultCode::ENCODER_1_DIRECTION:
            return "ENCODER_1_DIRECTION";

        case FaultCode::ENCODER_2_DIRECTION:
            return "ENCODER_2_DIRECTION";

        case FaultCode::SOFTWARE_LIMIT_1:
            return "SOFTWARE_LIMIT_1";

        case FaultCode::SOFTWARE_LIMIT_2:
            return "SOFTWARE_LIMIT_2";

        case FaultCode::HOMING_TIMEOUT:
            return "HOMING_TIMEOUT";

        case FaultCode::NVS_WRITE_ERROR:
            return "NVS_WRITE_ERROR";

        default:
            return "UNKNOWN";
    }
}

bool allAxesIdle()
{
    return
        axes[AZIMUTH_AXIS].mode == AxisMode::IDLE &&
        axes[ELEVATION_AXIS].mode == AxisMode::IDLE;
}

bool anyAxisMoving()
{
    return !allAxesIdle();
}

bool trackingOperational()
{
    return
        trackRequested &&
        positionCalibrated &&
        rtcValid &&
        rtcBackupConfigured &&
        locationValid &&
        !faultActive &&
        homingPhase == HomingPhase::IDLE;
}

// ============================================================
// SALIDA USB Y TELNET
// ============================================================

void remotePrintf(const char* format, ...)
{
    char buffer[512];

    va_list arguments;
    va_start(arguments, format);

    vsnprintf(
        buffer,
        sizeof(buffer),
        format,
        arguments
    );

    va_end(arguments);

    Serial.print(buffer);

    if (
        telnetClient &&
        telnetClient.connected()
    )
    {
        telnetClient.print(buffer);
    }
}

void remotePrintln(const char* text)
{
    Serial.println(text);

    if (
        telnetClient &&
        telnetClient.connected()
    )
    {
        telnetClient.println(text);
    }
}

void printPrompt()
{
    if (
        telnetClient &&
        telnetClient.connected()
    )
    {
        telnetClient.print("> ");
    }
}

// ============================================================
// ENCODERS
// ============================================================

void ARDUINO_ISR_ATTR encoder1Interrupt()
{
    portENTER_CRITICAL_ISR(&encoderMux);

    const uint8_t currentState =
        (
            gpio_get_level(
                static_cast<gpio_num_t>(
                    ENCODER_1_A_PIN
                )
            ) << 1
        ) |
        gpio_get_level(
            static_cast<gpio_num_t>(
                ENCODER_1_B_PIN
            )
        );

    const uint8_t transition =
        (
            (
                previousEncoderStates[0] &
                0x03U
            ) << 2
        ) |
        (
            currentState &
            0x03U
        );

    encoderCounts[0] +=
        static_cast<int64_t>(
            ENCODER_1_SIGN
        ) *
        QUADRATURE_TABLE[transition];

    previousEncoderStates[0] =
        currentState;

    portEXIT_CRITICAL_ISR(&encoderMux);
}

void ARDUINO_ISR_ATTR encoder2Interrupt()
{
    portENTER_CRITICAL_ISR(&encoderMux);

    const uint8_t currentState =
        (
            gpio_get_level(
                static_cast<gpio_num_t>(
                    ENCODER_2_A_PIN
                )
            ) << 1
        ) |
        gpio_get_level(
            static_cast<gpio_num_t>(
                ENCODER_2_B_PIN
            )
        );

    const uint8_t transition =
        (
            (
                previousEncoderStates[1] &
                0x03U
            ) << 2
        ) |
        (
            currentState &
            0x03U
        );

    encoderCounts[1] +=
        static_cast<int64_t>(
            ENCODER_2_SIGN
        ) *
        QUADRATURE_TABLE[transition];

    previousEncoderStates[1] =
        currentState;

    portEXIT_CRITICAL_ISR(&encoderMux);
}

int64_t readEncoderCount(uint8_t axis)
{
    int64_t result;

    portENTER_CRITICAL(&encoderMux);

    result = encoderCounts[axis];

    portEXIT_CRITICAL(&encoderMux);

    return result;
}

void setEncoderCount(
    uint8_t axis,
    int64_t value
)
{
    portENTER_CRITICAL(&encoderMux);

    encoderCounts[axis] = value;

    portEXIT_CRITICAL(&encoderMux);
}

double countsToAntennaDegrees(int64_t counts)
{
    return
        static_cast<double>(counts) *
        360.0 /
        ANTENNA_COUNTS_PER_REVOLUTION;
}

int64_t antennaDegreesToCounts(double degrees)
{
    return static_cast<int64_t>(
        llround(
            degrees *
            ANTENNA_COUNTS_PER_REVOLUTION /
            360.0
        )
    );
}

double currentAxisDegrees(uint8_t axis)
{
    return countsToAntennaDegrees(
        readEncoderCount(axis)
    );
}

bool targetInsideLimits(
    uint8_t axis,
    double targetDegrees
)
{
    if (!isfinite(targetDegrees))
    {
        return false;
    }

    if (axis == AZIMUTH_AXIS)
    {
        return
            targetDegrees >= AZIMUTH_MIN_DEGREES &&
            targetDegrees <= AZIMUTH_MAX_DEGREES;
    }

    return
        targetDegrees >= ELEVATION_MIN_DEGREES &&
        targetDegrees <= ELEVATION_MAX_DEGREES;
}

// ============================================================
// FINALES DE CARRERA NC + INPUT_PULLUP
// ============================================================

void initializeLimitInputs()
{
    for (
        uint8_t axis = 0;
        axis < AXIS_COUNT;
        ++axis
    )
    {
        pinMode(
            LIMIT_PINS[axis],
            INPUT_PULLUP
        );

        const bool active =
            digitalRead(
                LIMIT_PINS[axis]
            ) == HIGH;

        limits[axis].rawActive =
            active;

        limits[axis].stableActive =
            active;

        limits[axis].rawChangedMs =
            millis();
    }
}

void serviceLimitInputs()
{
    const uint32_t now =
        millis();

    for (
        uint8_t axis = 0;
        axis < AXIS_COUNT;
        ++axis
    )
    {
        const bool raw =
            digitalRead(
                LIMIT_PINS[axis]
            ) == HIGH;

        if (
            raw !=
            limits[axis].rawActive
        )
        {
            limits[axis].rawActive =
                raw;

            limits[axis].rawChangedMs =
                now;
        }

        if (
            now -
            limits[axis].rawChangedMs >=
            LIMIT_DEBOUNCE_MS
        )
        {
            limits[axis].stableActive =
                limits[axis].rawActive;
        }
    }
}

bool limitActive(uint8_t axis)
{
    return limits[axis].stableActive;
}

// ============================================================
// CRC Y NVS
// ============================================================

uint32_t calculateCrc32(
    const uint8_t* data,
    size_t length
)
{
    uint32_t crc = 0xFFFFFFFFUL;

    for (
        size_t index = 0;
        index < length;
        ++index
    )
    {
        crc ^= data[index];

        for (
            uint8_t bit = 0;
            bit < 8;
            ++bit
        )
        {
            const uint32_t mask =
                static_cast<uint32_t>(
                    -static_cast<int32_t>(
                        crc & 1U
                    )
                );

            crc =
                (crc >> 1U) ^
                (
                    0xEDB88320UL &
                    mask
                );
        }
    }

    return ~crc;
}

bool sequenceIsNewer(
    uint32_t first,
    uint32_t second
)
{
    return
        static_cast<int32_t>(
            first - second
        ) >
        0;
}

bool validRecordV2(
    const PersistentRecordV2& record
)
{
    if (
        record.magic != PERSISTENT_MAGIC ||
        record.version != PERSISTENT_VERSION_V2 ||
        record.size != sizeof(PersistentRecordV2)
    )
    {
        return false;
    }

    return
        calculateCrc32(
            reinterpret_cast<const uint8_t*>(
                &record
            ),
            offsetof(
                PersistentRecordV2,
                crc32
            )
        ) ==
        record.crc32;
}

bool validRecordV1(
    const PersistentRecordV1& record
)
{
    if (
        record.magic != PERSISTENT_MAGIC ||
        record.version != PERSISTENT_VERSION_V1 ||
        record.size != sizeof(PersistentRecordV1)
    )
    {
        return false;
    }

    return
        calculateCrc32(
            reinterpret_cast<const uint8_t*>(
                &record
            ),
            offsetof(
                PersistentRecordV1,
                crc32
            )
        ) ==
        record.crc32;
}

bool readSlotV2(
    Preferences& preferences,
    const char* key,
    PersistentRecordV2& record
)
{
    if (
        preferences.getBytesLength(key) !=
        sizeof(PersistentRecordV2)
    )
    {
        return false;
    }

    return
        preferences.getBytes(
            key,
            &record,
            sizeof(record)
        ) ==
            sizeof(record) &&
        validRecordV2(record);
}

bool readSlotV1(
    Preferences& preferences,
    const char* key,
    PersistentRecordV1& record
)
{
    if (
        preferences.getBytesLength(key) !=
        sizeof(PersistentRecordV1)
    )
    {
        return false;
    }

    return
        preferences.getBytes(
            key,
            &record,
            sizeof(record)
        ) ==
            sizeof(record) &&
        validRecordV1(record);
}

PersistentRecordV2 buildPersistentRecord(
    uint32_t sequence
)
{
    PersistentRecordV2 record = {};

    record.magic =
        PERSISTENT_MAGIC;

    record.version =
        PERSISTENT_VERSION_V2;

    record.size =
        sizeof(PersistentRecordV2);

    record.sequence =
        sequence;

    record.locationValid =
        locationValid ? 1U : 0U;

    record.trackRequested =
        trackRequested ? 1U : 0U;

    record.rtcBackupEverProven =
        rtcBackupEverProven ? 1U : 0U;

    record.latitudeDegrees =
        latitudeDegrees;

    record.longitudeDegrees =
        longitudeDegrees;

    record.altitudeMeters =
        altitudeMeters;

    record.crc32 =
        calculateCrc32(
            reinterpret_cast<const uint8_t*>(
                &record
            ),
            offsetof(
                PersistentRecordV2,
                crc32
            )
        );

    return record;
}

bool persistentPayloadMatches(
    const PersistentRecordV2& record
)
{
    return
        record.locationValid ==
            (locationValid ? 1U : 0U) &&

        record.trackRequested ==
            (trackRequested ? 1U : 0U) &&

        record.rtcBackupEverProven ==
            (rtcBackupEverProven ? 1U : 0U) &&

        record.latitudeDegrees ==
            latitudeDegrees &&

        record.longitudeDegrees ==
            longitudeDegrees &&

        record.altitudeMeters ==
            altitudeMeters;
}

bool verifyPersistentSlot(
    const char* key,
    const PersistentRecordV2& expected
)
{
    Preferences preferences;

    if (
        !preferences.begin(
            PREFERENCES_NAMESPACE,
            true
        )
    )
    {
        return false;
    }

    PersistentRecordV2 actual = {};

    const bool ok =
        readSlotV2(
            preferences,
            key,
            actual
        );

    preferences.end();

    return
        ok &&
        memcmp(
            &actual,
            &expected,
            sizeof(expected)
        ) == 0;
}

bool savePersistentConfig(bool forceWrite)
{
    if (
        !forceWrite &&
        lastCommittedRecordValid &&
        persistentPayloadMatches(
            lastCommittedRecord
        )
    )
    {
        return true;
    }

    const PersistentRecordV2 record =
        buildPersistentRecord(
            persistentSequence + 1U
        );

    const char* key =
        (
            record.sequence &
            1U
        )
            ? PERSISTENT_KEY_A
            : PERSISTENT_KEY_B;

    Preferences preferences;

    if (
        !preferences.begin(
            PREFERENCES_NAMESPACE,
            false
        )
    )
    {
        return false;
    }

    const size_t written =
        preferences.putBytes(
            key,
            &record,
            sizeof(record)
        );

    preferences.end();

    if (
        written != sizeof(record) ||
        !verifyPersistentSlot(
            key,
            record
        )
    )
    {
        return false;
    }

    persistentSequence =
        record.sequence;

    lastCommittedRecord =
        record;

    lastCommittedRecordValid =
        true;

    return true;
}

bool validateLoadedLocation()
{
    return
        isfinite(latitudeDegrees) &&
        isfinite(longitudeDegrees) &&
        isfinite(altitudeMeters) &&

        latitudeDegrees >= -90.0 &&
        latitudeDegrees <= 90.0 &&

        longitudeDegrees >= -180.0 &&
        longitudeDegrees <= 180.0 &&

        altitudeMeters >= -500.0 &&
        altitudeMeters <= 10000.0;
}

bool loadPersistentConfig()
{
    Preferences preferences;

    if (
        !preferences.begin(
            PREFERENCES_NAMESPACE,
            true
        )
    )
    {
        return false;
    }

    PersistentRecordV2 a2 = {};
    PersistentRecordV2 b2 = {};

    const bool validA2 =
        readSlotV2(
            preferences,
            PERSISTENT_KEY_A,
            a2
        );

    const bool validB2 =
        readSlotV2(
            preferences,
            PERSISTENT_KEY_B,
            b2
        );

    if (validA2 || validB2)
    {
        const PersistentRecordV2& selected =
            validA2 && validB2
                ? (
                    sequenceIsNewer(
                        a2.sequence,
                        b2.sequence
                    )
                        ? a2
                        : b2
                )
                : (
                    validA2
                        ? a2
                        : b2
                );

        preferences.end();

        persistentSequence =
            selected.sequence;

        lastCommittedRecord =
            selected;

        lastCommittedRecordValid =
            true;

        locationValid =
            selected.locationValid != 0;

        trackRequested =
            selected.trackRequested != 0;

        rtcBackupEverProven =
            selected.rtcBackupEverProven != 0;

        latitudeDegrees =
            selected.latitudeDegrees;

        longitudeDegrees =
            selected.longitudeDegrees;

        altitudeMeters =
            selected.altitudeMeters;

        if (!validateLoadedLocation())
        {
            locationValid = false;
        }

        return true;
    }

    // Migracion automatica desde la version anterior.
    PersistentRecordV1 a1 = {};
    PersistentRecordV1 b1 = {};

    const bool validA1 =
        readSlotV1(
            preferences,
            PERSISTENT_KEY_A,
            a1
        );

    const bool validB1 =
        readSlotV1(
            preferences,
            PERSISTENT_KEY_B,
            b1
        );

    preferences.end();

    if (!validA1 && !validB1)
    {
        return false;
    }

    const PersistentRecordV1& selected =
        validA1 && validB1
            ? (
                sequenceIsNewer(
                    a1.sequence,
                    b1.sequence
                )
                    ? a1
                    : b1
            )
            : (
                validA1
                    ? a1
                    : b1
            );

    persistentSequence =
        selected.sequence;

    locationValid =
        selected.locationValid != 0;

    trackRequested =
        selected.trackEnabled != 0;

    rtcBackupEverProven =
        (
            selected.flags &
            0x01U
        ) != 0;

    latitudeDegrees =
        selected.latitudeDegrees;

    longitudeDegrees =
        selected.longitudeDegrees;

    altitudeMeters =
        selected.altitudeMeters;

    if (!validateLoadedLocation())
    {
        locationValid = false;
    }

    lastCommittedRecordValid = false;

    return savePersistentConfig(true);
}

bool setTrackRequestedPersisted(
    bool enabled,
    bool report
)
{
    const bool oldValue =
        trackRequested;

    trackRequested =
        enabled;

    if (!savePersistentConfig(true))
    {
        trackRequested =
            oldValue;

        remotePrintln(
            "ERROR NVS: no se pudo guardar TRACK."
        );

        return false;
    }

    if (report)
    {
        remotePrintf(
            "TRACK solicitado %s guardado y "
            "verificado en NVS.\n",
            enabled ? "ON" : "OFF"
        );
    }

    return true;
}

// ============================================================
// RTC: I2C Y EEPROM
// ============================================================

bool rtcWriteRegisters(
    uint8_t startRegister,
    const uint8_t* data,
    size_t length
)
{
    Wire.beginTransmission(
        RV3028_ADDRESS
    );

    Wire.write(startRegister);

    for (
        size_t index = 0;
        index < length;
        ++index
    )
    {
        Wire.write(data[index]);
    }

    return
        Wire.endTransmission() == 0;
}

bool rtcWriteRegister(
    uint8_t registerAddress,
    uint8_t value
)
{
    return rtcWriteRegisters(
        registerAddress,
        &value,
        1
    );
}

bool rtcReadRegisters(
    uint8_t startRegister,
    uint8_t* data,
    size_t length
)
{
    Wire.beginTransmission(
        RV3028_ADDRESS
    );

    Wire.write(startRegister);

    if (
        Wire.endTransmission(false) != 0
    )
    {
        return false;
    }

    const size_t received =
        Wire.requestFrom(
            RV3028_ADDRESS,
            static_cast<uint8_t>(
                length
            )
        );

    if (received != length)
    {
        return false;
    }

    for (
        size_t index = 0;
        index < length;
        ++index
    )
    {
        data[index] =
            static_cast<uint8_t>(
                Wire.read()
            );
    }

    return true;
}

bool rtcReadRegister(
    uint8_t registerAddress,
    uint8_t& value
)
{
    return rtcReadRegisters(
        registerAddress,
        &value,
        1
    );
}

bool rtcProbe()
{
    Wire.beginTransmission(
        RV3028_ADDRESS
    );

    return
        Wire.endTransmission() == 0;
}

bool rtcWaitForEeprom(uint32_t timeoutMs)
{
    const uint32_t start =
        millis();

    while (
        millis() - start <
        timeoutMs
    )
    {
        uint8_t status = 0;

        if (
            !rtcReadRegister(
                RTC_REG_STATUS,
                status
            )
        )
        {
            return false;
        }

        if (
            (
                status &
                RTC_STATUS_EEBUSY
            ) == 0
        )
        {
            return true;
        }

        delay(2);
    }

    return false;
}

bool rtcBeginDirectEepromAccess(
    uint8_t& originalControl1,
    uint8_t& originalBackupRam
)
{
    if (
        !rtcReadRegister(
            RTC_REG_CONTROL_1,
            originalControl1
        )
    )
    {
        return false;
    }

    if (
        !rtcReadRegister(
            RTC_REG_BACKUP_RAM,
            originalBackupRam
        )
    )
    {
        return false;
    }

    const uint8_t disabledBackup =
        originalBackupRam &
        static_cast<uint8_t>(
            ~RTC_BACKUP_BSM_MASK
        );

    if (
        !rtcWriteRegister(
            RTC_REG_BACKUP_RAM,
            disabledBackup
        )
    )
    {
        return false;
    }

    if (
        !rtcWriteRegister(
            RTC_REG_CONTROL_1,
            originalControl1 |
            RTC_CONTROL_1_EERD
        )
    )
    {
        rtcWriteRegister(
            RTC_REG_BACKUP_RAM,
            originalBackupRam
        );

        return false;
    }

    if (!rtcWaitForEeprom(1000))
    {
        rtcWriteRegister(
            RTC_REG_CONTROL_1,
            originalControl1 &
            static_cast<uint8_t>(
                ~RTC_CONTROL_1_EERD
            )
        );

        rtcWriteRegister(
            RTC_REG_BACKUP_RAM,
            originalBackupRam
        );

        return false;
    }

    return true;
}

void rtcEndDirectEepromAccess(
    uint8_t originalControl1,
    uint8_t finalBackupRam
)
{
    rtcWriteRegister(
        RTC_REG_BACKUP_RAM,
        finalBackupRam
    );

    rtcWriteRegister(
        RTC_REG_CONTROL_1,
        originalControl1 &
        static_cast<uint8_t>(
            ~RTC_CONTROL_1_EERD
        )
    );
}

bool rtcReadEepromByteLocked(
    uint8_t address,
    uint8_t& value
)
{
    if (!rtcWaitForEeprom(1000))
    {
        return false;
    }

    if (
        !rtcWriteRegister(
            RTC_REG_EE_ADDRESS,
            address
        )
    )
    {
        return false;
    }

    if (
        !rtcWriteRegister(
            RTC_REG_EE_COMMAND,
            RTC_EE_COMMAND_FIRST
        )
    )
    {
        return false;
    }

    if (
        !rtcWriteRegister(
            RTC_REG_EE_COMMAND,
            RTC_EE_COMMAND_READ_ONE
        )
    )
    {
        return false;
    }

    delay(2);

    if (!rtcWaitForEeprom(1000))
    {
        return false;
    }

    return rtcReadRegister(
        RTC_REG_EE_DATA,
        value
    );
}

bool rtcWriteEepromByteLocked(
    uint8_t address,
    uint8_t value
)
{
    if (!rtcWaitForEeprom(1000))
    {
        return false;
    }

    if (
        !rtcWriteRegister(
            RTC_REG_EE_ADDRESS,
            address
        )
    )
    {
        return false;
    }

    if (
        !rtcWriteRegister(
            RTC_REG_EE_DATA,
            value
        )
    )
    {
        return false;
    }

    if (
        !rtcWriteRegister(
            RTC_REG_EE_COMMAND,
            RTC_EE_COMMAND_FIRST
        )
    )
    {
        return false;
    }

    if (
        !rtcWriteRegister(
            RTC_REG_EE_COMMAND,
            RTC_EE_COMMAND_WRITE_ONE
        )
    )
    {
        return false;
    }

    delay(12);

    return rtcWaitForEeprom(1500);
}

bool rtcReadEepromByte(
    uint8_t address,
    uint8_t& value
)
{
    uint8_t control1 = 0;
    uint8_t backupRam = 0;

    if (
        !rtcBeginDirectEepromAccess(
            control1,
            backupRam
        )
    )
    {
        return false;
    }

    const bool success =
        rtcReadEepromByteLocked(
            address,
            value
        );

    rtcEndDirectEepromAccess(
        control1,
        backupRam
    );

    return success;
}

bool rtcBackupValueIsConfigured(uint8_t value)
{
    return
        (
            value &
            RTC_BACKUP_REQUIRED_MASK
        ) ==
        RTC_BACKUP_REQUIRED_VALUE;
}

uint8_t rtcBuildDesiredBackupValue(uint8_t original)
{
    return
        (
            original &
            (
                RTC_BACKUP_OFFSET_LSB |
                RTC_BACKUP_TCR_MASK
            )
        ) |
        RTC_BACKUP_REQUIRED_VALUE;
}

bool rtcConfigureBackupPersistent()
{
    rtcBackupConfigured = false;
    rtcBackupEepromReadable = false;
    rtcBackupRamMatchesEeprom = false;

    if (!rtcDetected)
    {
        return false;
    }

    uint8_t currentEeprom = 0;

    if (
        !rtcReadEepromByte(
            RTC_EEPROM_BACKUP_ADDRESS,
            currentEeprom
        )
    )
    {
        return false;
    }

    rtcBackupEepromReadable = true;
    rtcBackupEepromValue = currentEeprom;

    const uint8_t desired =
        rtcBuildDesiredBackupValue(
            currentEeprom
        );

    if (
        !rtcBackupValueIsConfigured(
            currentEeprom
        )
    )
    {
        uint8_t control1 = 0;
        uint8_t backupRam = 0;

        if (
            !rtcBeginDirectEepromAccess(
                control1,
                backupRam
            )
        )
        {
            return false;
        }

        bool success =
            rtcWriteEepromByteLocked(
                RTC_EEPROM_BACKUP_ADDRESS,
                desired
            );

        uint8_t verify = 0;

        if (success)
        {
            success =
                rtcReadEepromByteLocked(
                    RTC_EEPROM_BACKUP_ADDRESS,
                    verify
                );
        }

        if (
            success &&
            verify == desired
        )
        {
            rtcEndDirectEepromAccess(
                control1,
                desired
            );

            rtcBackupEepromValue =
                verify;
        }
        else
        {
            rtcEndDirectEepromAccess(
                control1,
                backupRam
            );

            return false;
        }
    }
    else
    {
        if (
            !rtcWriteRegister(
                RTC_REG_BACKUP_RAM,
                desired
            )
        )
        {
            return false;
        }
    }

    uint8_t finalEeprom = 0;
    uint8_t finalRam = 0;

    const bool eepromOk =
        rtcReadEepromByte(
            RTC_EEPROM_BACKUP_ADDRESS,
            finalEeprom
        );

    const bool ramOk =
        rtcReadRegister(
            RTC_REG_BACKUP_RAM,
            finalRam
        );

    rtcBackupEepromReadable =
        eepromOk;

    if (eepromOk)
    {
        rtcBackupEepromValue =
            finalEeprom;
    }

    if (ramOk)
    {
        rtcBackupRamValue =
            finalRam;
    }

    rtcBackupRamMatchesEeprom =
        eepromOk &&
        ramOk &&
        finalEeprom == finalRam;

    rtcBackupConfigured =
        eepromOk &&
        ramOk &&
        rtcBackupValueIsConfigured(
            finalEeprom
        ) &&
        rtcBackupValueIsConfigured(
            finalRam
        );

    return rtcBackupConfigured;
}

// ============================================================
// RTC: FECHA Y HORA
// ============================================================

uint8_t decimalToBcd(uint8_t value)
{
    return
        (
            (value / 10U) << 4U
        ) |
        (
            value % 10U
        );
}

uint8_t bcdToDecimal(uint8_t value)
{
    return
        (
            (value >> 4U) *
            10U
        ) +
        (
            value &
            0x0FU
        );
}

bool isLeapYear(uint16_t year)
{
    return
        (
            year % 4U == 0U &&
            year % 100U != 0U
        ) ||
        year % 400U == 0U;
}

uint8_t daysInMonth(
    uint16_t year,
    uint8_t month
)
{
    static const uint8_t DAYS[12] = {
        31, 28, 31, 30,
        31, 30, 31, 31,
        30, 31, 30, 31
    };

    if (
        month < 1 ||
        month > 12
    )
    {
        return 0;
    }

    if (
        month == 2 &&
        isLeapYear(year)
    )
    {
        return 29;
    }

    return DAYS[month - 1];
}

bool validateDateTime(
    const RtcDateTime& dt
)
{
    return
        dt.year >= 2000 &&
        dt.year <= 2099 &&

        dt.month >= 1 &&
        dt.month <= 12 &&

        dt.day >= 1 &&
        dt.day <=
            daysInMonth(
                dt.year,
                dt.month
            ) &&

        dt.hour <= 23 &&
        dt.minute <= 59 &&
        dt.second <= 59;
}

uint8_t calculateWeekday(
    uint16_t year,
    uint8_t month,
    uint8_t day
)
{
    static const uint8_t ADJ[12] = {
        0, 3, 2, 5,
        0, 3, 5, 1,
        4, 6, 2, 4
    };

    if (month < 3)
    {
        --year;
    }

    return
        (
            year +
            year / 4U -
            year / 100U +
            year / 400U +
            ADJ[month - 1] +
            day
        ) %
        7U;
}

int64_t dateTimeToUnixSeconds(
    const RtcDateTime& dt
)
{
    int64_t days = 0;

    for (
        uint16_t year = 1970;
        year < dt.year;
        ++year
    )
    {
        days +=
            isLeapYear(year)
                ? 366
                : 365;
    }

    for (
        uint8_t month = 1;
        month < dt.month;
        ++month
    )
    {
        days +=
            daysInMonth(
                dt.year,
                month
            );
    }

    days +=
        static_cast<int64_t>(
            dt.day
        ) -
        1;

    return
        days * 86400LL +
        static_cast<int64_t>(
            dt.hour
        ) * 3600LL +
        static_cast<int64_t>(
            dt.minute
        ) * 60LL +
        dt.second;
}

bool rtcEnsure24HourMode()
{
    uint8_t control2 = 0;

    if (
        !rtcReadRegister(
            RTC_REG_CONTROL_2,
            control2
        )
    )
    {
        return false;
    }

    if (
        (
            control2 &
            RTC_CONTROL_2_12_24
        ) == 0
    )
    {
        return true;
    }

    control2 &=
        static_cast<uint8_t>(
            ~RTC_CONTROL_2_12_24
        );

    return rtcWriteRegister(
        RTC_REG_CONTROL_2,
        control2
    );
}

bool rtcReadDateTime(RtcDateTime& dt)
{
    uint8_t registers[7];

    if (
        !rtcReadRegisters(
            RTC_REG_SECONDS,
            registers,
            sizeof(registers)
        )
    )
    {
        return false;
    }

    dt.second =
        bcdToDecimal(
            registers[0] &
            0x7F
        );

    dt.minute =
        bcdToDecimal(
            registers[1] &
            0x7F
        );

    dt.hour =
        bcdToDecimal(
            registers[2] &
            0x3F
        );

    dt.weekday =
        registers[3] &
        0x07;

    dt.day =
        bcdToDecimal(
            registers[4] &
            0x3F
        );

    dt.month =
        bcdToDecimal(
            registers[5] &
            0x1F
        );

    dt.year =
        2000U +
        bcdToDecimal(
            registers[6]
        );

    return validateDateTime(dt);
}

bool rtcCheckValidity()
{
    if (!rtcDetected)
    {
        return false;
    }

    uint8_t status = 0;

    if (
        !rtcReadRegister(
            RTC_REG_STATUS,
            status
        )
    )
    {
        return false;
    }

    if (
        (
            status &
            RTC_STATUS_PORF
        ) != 0
    )
    {
        return false;
    }

    RtcDateTime dt;

    return rtcReadDateTime(dt);
}

bool rtcSetDateTimeVerified(
    const RtcDateTime& dt
)
{
    if (
        !rtcDetected ||
        !validateDateTime(dt)
    )
    {
        return false;
    }

    if (
        !rtcConfigureBackupPersistent() ||
        !rtcEnsure24HourMode()
    )
    {
        return false;
    }

    const uint8_t data[7] = {
        decimalToBcd(dt.second),
        decimalToBcd(dt.minute),
        decimalToBcd(dt.hour),
        dt.weekday,
        decimalToBcd(dt.day),
        decimalToBcd(dt.month),
        decimalToBcd(
            static_cast<uint8_t>(
                dt.year -
                2000U
            )
        )
    };

    if (
        !rtcWriteRegisters(
            RTC_REG_SECONDS,
            data,
            sizeof(data)
        )
    )
    {
        return false;
    }

    delay(30);

    RtcDateTime readBack;

    if (!rtcReadDateTime(readBack))
    {
        return false;
    }

    if (
        llabs(
            dateTimeToUnixSeconds(readBack) -
            dateTimeToUnixSeconds(dt)
        ) >
        2
    )
    {
        return false;
    }

    uint8_t status = 0;

    if (
        !rtcReadRegister(
            RTC_REG_STATUS,
            status
        )
    )
    {
        return false;
    }

    status &=
        static_cast<uint8_t>(
            ~RTC_STATUS_PORF
        );

    if (
        !rtcWriteRegister(
            RTC_REG_STATUS,
            status
        )
    )
    {
        return false;
    }

    rtcValid =
        rtcCheckValidity();

    forceSolarUpdate = true;

    return
        rtcValid &&
        rtcBackupConfigured;
}

void rtcCaptureBackupEvidence()
{
    uint8_t status = 0;

    if (
        !rtcReadRegister(
            RTC_REG_STATUS,
            status
        )
    )
    {
        return;
    }

    rtcBootStatusRaw =
        status;

    rtcBackupUsedThisBoot =
        (
            status &
            RTC_STATUS_BSF
        ) != 0;

    if (rtcBackupUsedThisBoot)
    {
        const bool wasProven =
            rtcBackupEverProven;

        rtcBackupEverProven =
            true;

        status &=
            static_cast<uint8_t>(
                ~RTC_STATUS_BSF
            );

        rtcWriteRegister(
            RTC_REG_STATUS,
            status
        );

        if (!wasProven)
        {
            savePersistentConfig(true);
        }
    }
}

void setupRtc()
{
    pinMode(
        RTC_CLKOUT_PIN,
        INPUT
    );

    pinMode(
        RTC_INT_PIN,
        INPUT_PULLUP
    );

    pinMode(
        RTC_EVI_PIN,
        OUTPUT
    );

    digitalWrite(
        RTC_EVI_PIN,
        LOW
    );

    Wire.begin(
        RTC_SDA_PIN,
        RTC_SCL_PIN
    );

    Wire.setClock(100000);

    delay(250);

    rtcDetected =
        rtcProbe();

    if (!rtcDetected)
    {
        Serial.println(
            "ERROR: RV-3028-C7 no detectado."
        );

        return;
    }

    if (!rtcWaitForEeprom(1000))
    {
        Serial.println(
            "ERROR: EEPROM del RTC ocupada."
        );

        return;
    }

    rtcCaptureBackupEvidence();

    const bool mode24Ok =
        rtcEnsure24HourMode();

    const bool backupOk =
        rtcConfigureBackupPersistent();

    rtcValid =
        mode24Ok &&
        rtcCheckValidity();

    Serial.printf(
        "RTC detectado. Hora valida: %s | "
        "EEPROM LSM: %s | BSF: %s\n",
        rtcValid ? "SI" : "NO",
        backupOk ? "SI" : "NO",
        rtcBackupUsedThisBoot
            ? "SI"
            : "NO"
    );
}

void printRtcTime()
{
    if (!rtcDetected)
    {
        remotePrintln(
            "RTC no detectado."
        );

        return;
    }

    RtcDateTime dt;

    if (!rtcReadDateTime(dt))
    {
        remotePrintln(
            "No se pudo leer el RTC."
        );

        return;
    }

    uint8_t status = 0;

    rtcReadRegister(
        RTC_REG_STATUS,
        status
    );

    remotePrintf(
        "UTC: %04u-%02u-%02u "
        "%02u:%02u:%02u | "
        "valido=%s | PORF=%u\n",
        dt.year,
        dt.month,
        dt.day,
        dt.hour,
        dt.minute,
        dt.second,
        rtcValid ? "SI" : "NO",
        (
            status &
            RTC_STATUS_PORF
        ) != 0
    );
}

void printRtcStatus()
{
    remotePrintln(
        "\n========== RTC =========="
    );

    remotePrintf(
        "RTC detectado: %s\n",
        rtcDetected ? "SI" : "NO"
    );

    if (!rtcDetected)
    {
        remotePrintln(
            "=========================\n"
        );

        return;
    }

    uint8_t status = 0;
    uint8_t ram = 0;
    uint8_t eeprom = 0;

    const bool statusOk =
        rtcReadRegister(
            RTC_REG_STATUS,
            status
        );

    const bool ramOk =
        rtcReadRegister(
            RTC_REG_BACKUP_RAM,
            ram
        );

    const bool eepromOk =
        rtcReadEepromByte(
            RTC_EEPROM_BACKUP_ADDRESS,
            eeprom
        );

    if (!statusOk || !ramOk)
    {
        remotePrintln(
            "Error leyendo RTC.\n"
            "=========================\n"
        );

        return;
    }

    rtcBackupRamValue =
        ram;

    rtcBackupEepromReadable =
        eepromOk;

    if (eepromOk)
    {
        rtcBackupEepromValue =
            eeprom;
    }

    rtcBackupRamMatchesEeprom =
        eepromOk &&
        ram == eeprom;

    rtcBackupConfigured =
        eepromOk &&
        rtcBackupValueIsConfigured(ram) &&
        rtcBackupValueIsConfigured(eeprom);

    remotePrintf(
        "Hora UTC valida: %s\n",
        rtcValid ? "SI" : "NO"
    );

    remotePrintf(
        "PORF actual: %u\n",
        (
            status &
            RTC_STATUS_PORF
        ) != 0
    );

    remotePrintf(
        "BSF detectado al arrancar: %s\n",
        rtcBackupUsedThisBoot
            ? "SI"
            : "NO"
    );

    remotePrintf(
        "STATUS al arrancar: 0x%02X\n",
        rtcBootStatusRaw
    );

    remotePrintf(
        "BACKUP RAM: 0x%02X\n",
        ram
    );

    if (eepromOk)
    {
        remotePrintf(
            "BACKUP EEPROM real: 0x%02X\n",
            eeprom
        );
    }
    else
    {
        remotePrintln(
            "BACKUP EEPROM real: ERROR"
        );
    }

    remotePrintf(
        "RAM coincide con EEPROM: %s\n",
        rtcBackupRamMatchesEeprom
            ? "SI"
            : "NO"
    );

    remotePrintf(
        "Modo RAM: %s\n",
        (
            ram &
            RTC_BACKUP_BSM_MASK
        ) ==
        RTC_BACKUP_BSM_LSM
            ? "LSM"
            : "NO LSM"
    );

    remotePrintf(
        "Modo EEPROM: %s\n",
        eepromOk &&
        (
            eeprom &
            RTC_BACKUP_BSM_MASK
        ) ==
        RTC_BACKUP_BSM_LSM
            ? "LSM"
            : "NO LSM/ERROR"
    );

    remotePrintf(
        "FEDE EEPROM: %s\n",
        eepromOk &&
        (
            eeprom &
            RTC_BACKUP_FEDE
        )
            ? "ACTIVADO"
            : "NO"
    );

    remotePrintf(
        "Cargador interno: %s\n",
        eepromOk &&
        (
            eeprom &
            RTC_BACKUP_TCE
        )
            ? "ACTIVADO"
            : "DESACTIVADO"
    );

    remotePrintf(
        "Respaldo persistente verificado: %s\n",
        rtcBackupConfigured
            ? "SI"
            : "NO"
    );

    remotePrintf(
        "Fuente de respaldo comprobada alguna vez: %s\n",
        rtcBackupEverProven
            ? "SI"
            : "NO"
    );

    remotePrintln(
        "=========================\n"
    );
}

// ============================================================
// MOTORES
// ============================================================

uint8_t profileMaximumPwm(MotionProfile profile)
{
    switch (profile)
    {
        case MotionProfile::PARK:
            return PARK_MAX_PWM;

        case MotionProfile::TRACK:
            return TRACK_MAX_PWM;

        case MotionProfile::MANUAL_GOTO:
        case MotionProfile::MANUAL_JOG:
            return MANUAL_MAX_PWM;

        default:
            return SLEW_MAX_PWM;
    }
}

double profileSlowdownDegrees(
    MotionProfile profile
)
{
    switch (profile)
    {
        case MotionProfile::PARK:
            return PARK_SLOWDOWN_DEGREES;

        case MotionProfile::TRACK:
            return TRACK_SLOWDOWN_DEGREES;

        case MotionProfile::MANUAL_GOTO:
        case MotionProfile::MANUAL_JOG:
            return MANUAL_SLOWDOWN_DEGREES;

        default:
            return SLEW_SLOWDOWN_DEGREES;
    }
}

uint8_t limitPwm(unsigned long pwm)
{
    return
        pwm > PWM_MAX
            ? PWM_MAX
            : static_cast<uint8_t>(pwm);
}

void writeAxisPwm(
    uint8_t axis,
    uint8_t pwm
)
{
    axes[axis].currentPwm =
        pwm;

    ledcWrite(
        MOTOR_PWM_PINS[axis],
        pwm
    );
}

void stopAxis(uint8_t axis)
{
    ledcWrite(
        MOTOR_PWM_PINS[axis],
        0
    );

    axes[axis].mode =
        AxisMode::IDLE;

    axes[axis].direction = 0;
    axes[axis].currentPwm = 0;
    axes[axis].maximumPwm = 0;
    axes[axis].limitReleaseAllowed = false;
}

void stopAllMotion(bool clearPurpose)
{
    stopAxis(AZIMUTH_AXIS);
    stopAxis(ELEVATION_AXIS);

    if (clearPurpose)
    {
        activePurpose =
            MovementPurpose::NONE;
    }
}

void configureAxisDirection(
    uint8_t axis,
    int8_t direction
)
{
    direction =
        direction >= 0
            ? 1
            : -1;

    ledcWrite(
        MOTOR_PWM_PINS[axis],
        0
    );

    axes[axis].currentPwm = 0;

    const uint8_t level =
        direction > 0
            ? MOTOR_POSITIVE_DIR_LEVEL[axis]
            : static_cast<uint8_t>(
                !MOTOR_POSITIVE_DIR_LEVEL[axis]
            );

    digitalWrite(
        MOTOR_DIR_PINS[axis],
        level
    );

    axes[axis].direction =
        direction;
}

void configureLimitReleaseAllowance(
    uint8_t axis,
    int8_t direction
)
{
    AxisState& state =
        axes[axis];

    const bool movingAway =
        direction ==
        -HOMING_TOWARD_LIMIT_DIRECTION[axis];

    state.limitReleaseAllowed =
        limitActive(axis) &&
        movingAway;

    state.limitReleaseStartCount =
        readEncoderCount(axis);

    state.limitReleaseStartMs =
        millis();
}

void rampAxisTowardPwm(
    uint8_t axis,
    uint8_t desiredPwm
)
{
    AxisState& state =
        axes[axis];

    const uint32_t now =
        millis();

    if (
        now -
        state.lastRampMs <
        PWM_RAMP_INTERVAL_MS
    )
    {
        return;
    }

    state.lastRampMs =
        now;

    uint8_t next =
        state.currentPwm;

    if (next < desiredPwm)
    {
        const uint16_t candidate =
            static_cast<uint16_t>(
                next
            ) +
            PWM_RAMP_UP_STEP;

        next =
            candidate > desiredPwm
                ? desiredPwm
                : static_cast<uint8_t>(
                    candidate
                );
    }
    else if (next > desiredPwm)
    {
        const int candidate =
            static_cast<int>(
                next
            ) -
            PWM_RAMP_DOWN_STEP;

        next =
            candidate < desiredPwm
                ? desiredPwm
                : static_cast<uint8_t>(
                    candidate
                );
    }

    if (
        next !=
        state.currentPwm
    )
    {
        writeAxisPwm(
            axis,
            next
        );
    }
}

uint8_t calculateDesiredPositionPwm(
    const AxisState& axis,
    double remainingDegrees
)
{
    const double slowdown =
        profileSlowdownDegrees(
            axis.profile
        );

    if (
        remainingDegrees >=
        slowdown
    )
    {
        return axis.maximumPwm;
    }

    const double ratio =
        clampDouble(
            remainingDegrees /
            slowdown,
            0.0,
            1.0
        );

    const double value =
        MIN_POSITION_PWM +
        (
            axis.maximumPwm -
            MIN_POSITION_PWM
        ) *
        ratio;

    return static_cast<uint8_t>(
        clampDouble(
            value,
            MIN_POSITION_PWM,
            axis.maximumPwm
        )
    );
}

bool prepareAxisAbsoluteMove(
    uint8_t axis,
    double targetDegrees,
    MotionProfile profile
)
{
    if (
        !targetInsideLimits(
            axis,
            targetDegrees
        )
    )
    {
        remotePrintf(
            "ERROR: objetivo de %s fuera "
            "de rango: %.3f.\n",
            axisName(axis),
            targetDegrees
        );

        return false;
    }

    const int64_t current =
        readEncoderCount(axis);

    const int64_t target =
        antennaDegreesToCounts(
            targetDegrees
        );

    const int64_t delta =
        target -
        current;

    int64_t tolerance =
        llabs(
            antennaDegreesToCounts(
                POSITION_TOLERANCE_DEGREES
            )
        );

    if (tolerance < 1)
    {
        tolerance = 1;
    }

    if (
        llabs(delta) <=
        tolerance
    )
    {
        stopAxis(axis);

        return false;
    }

    AxisState& state =
        axes[axis];

    state.mode =
        AxisMode::POSITION;

    state.profile =
        profile;

    state.startCount =
        current;

    state.targetCount =
        target;

    state.targetDegrees =
        targetDegrees;

    state.lastActivityCount =
        current;

    state.movementStartMs =
        millis();

    state.lastActivityMs =
        millis();

    state.lastRampMs =
        millis();

    state.maximumPwm =
        profileMaximumPwm(profile);

    const int8_t direction =
        delta > 0
            ? 1
            : -1;

    configureAxisDirection(
        axis,
        direction
    );

    configureLimitReleaseAllowance(
        axis,
        direction
    );

    return true;
}

bool startCoordinatedAbsoluteMove(
    double azimuth,
    double elevation,
    MovementPurpose purpose,
    MotionProfile profile
)
{
    if (!positionCalibrated)
    {
        remotePrintln(
            "ERROR: posicion no calibrada. "
            "Use CALIBRATE."
        );

        return false;
    }

    if (
        !targetInsideLimits(
            AZIMUTH_AXIS,
            azimuth
        ) ||
        !targetInsideLimits(
            ELEVATION_AXIS,
            elevation
        )
    )
    {
        remotePrintln(
            "ERROR: objetivo fuera de limites."
        );

        return false;
    }

    stopAllMotion(true);

    activePurpose =
        purpose;

    activeMotionProfile =
        profile;

    const bool azPrepared =
        prepareAxisAbsoluteMove(
            AZIMUTH_AXIS,
            azimuth,
            profile
        );

    const bool elPrepared =
        prepareAxisAbsoluteMove(
            ELEVATION_AXIS,
            elevation,
            profile
        );

    if (!azPrepared && !elPrepared)
    {
        activePurpose =
            MovementPurpose::NONE;

        if (
            purpose ==
            MovementPurpose::AUTO_PARK
        )
        {
            trackerState =
                TrackerState::PARKED;
        }
        else if (
            purpose ==
            MovementPurpose::AUTO_SUN
        )
        {
            trackerState =
                TrackerState::TRACKING;
        }

        return true;
    }

    if (
        purpose ==
        MovementPurpose::AUTO_SUN
    )
    {
        trackerState =
            TrackerState::SLEWING_TO_SUN;
    }
    else if (
        purpose ==
        MovementPurpose::AUTO_PARK
    )
    {
        trackerState =
            TrackerState::SLEWING_TO_PARK;
    }
    else
    {
        trackerState =
            TrackerState::MANUAL;
    }

    remotePrintf(
        "Movimiento absoluto: "
        "AZ=%.3f EL=%.3f\n",
        azimuth,
        elevation
    );

    return true;
}

FaultCode stuckFaultForAxis(uint8_t axis)
{
    return
        axis == AZIMUTH_AXIS
            ? FaultCode::LIMIT_1_STUCK
            : FaultCode::LIMIT_2_STUCK;
}

FaultCode notFoundFaultForAxis(uint8_t axis)
{
    return
        axis == AZIMUTH_AXIS
            ? FaultCode::LIMIT_1_NOT_FOUND
            : FaultCode::LIMIT_2_NOT_FOUND;
}

FaultCode stallFaultForAxis(uint8_t axis)
{
    return
        axis == AZIMUTH_AXIS
            ? FaultCode::ENCODER_1_STALL
            : FaultCode::ENCODER_2_STALL;
}

FaultCode directionFaultForAxis(uint8_t axis)
{
    return
        axis == AZIMUTH_AXIS
            ? FaultCode::ENCODER_1_DIRECTION
            : FaultCode::ENCODER_2_DIRECTION;
}

FaultCode softwareLimitFaultForAxis(uint8_t axis)
{
    return
        axis == AZIMUTH_AXIS
            ? FaultCode::SOFTWARE_LIMIT_1
            : FaultCode::SOFTWARE_LIMIT_2;
}

bool targetIsHomeLimit(
    uint8_t axis,
    double targetDegrees
)
{
    const double home =
        axis == AZIMUTH_AXIS
            ? HOME_AZIMUTH_DEGREES
            : HOME_ELEVATION_DEGREES;

    return
        fabs(
            targetDegrees -
            home
        ) <=
        0.20;
}

void updateAxis(uint8_t axis)
{
    AxisState& state =
        axes[axis];

    if (
        state.mode == AxisMode::IDLE ||
        state.mode == AxisMode::HOMING ||
        faultActive
    )
    {
        return;
    }

    const uint32_t now =
        millis();

    const int64_t current =
        readEncoderCount(axis);

    const double currentDegrees =
        countsToAntennaDegrees(
            current
        );

    if (positionCalibrated)
    {
        const double minimum =
            axis == AZIMUTH_AXIS
                ? AZIMUTH_MIN_DEGREES
                : ELEVATION_MIN_DEGREES;

        const double maximum =
            axis == AZIMUTH_AXIS
                ? AZIMUTH_MAX_DEGREES
                : ELEVATION_MAX_DEGREES;

        if (
            currentDegrees <
                minimum -
                SOFT_LIMIT_MARGIN_DEGREES ||
            currentDegrees >
                maximum +
                SOFT_LIMIT_MARGIN_DEGREES
        )
        {
            enterFault(
                softwareLimitFaultForAxis(axis),
                "%s fuera de limite: %.3f grados",
                axisName(axis),
                currentDegrees
            );

            return;
        }
    }

    if (limitActive(axis))
    {
        const int8_t toward =
            HOMING_TOWARD_LIMIT_DIRECTION[axis];

        if (state.direction == toward)
        {
            const double home =
                axis == AZIMUTH_AXIS
                    ? HOME_AZIMUTH_DEGREES
                    : HOME_ELEVATION_DEGREES;

            const double distanceFromHome =
                fabs(
                    currentDegrees -
                    home
                );

            const bool expectingHome =
                state.mode == AxisMode::JOG ||
                targetIsHomeLimit(
                    axis,
                    state.targetDegrees
                );

            if (
                expectingHome &&
                distanceFromHome <=
                LIMIT_HOME_ACCEPTANCE_DEGREES
            )
            {
                setEncoderCount(
                    axis,
                    antennaDegreesToCounts(
                        home
                    )
                );

                stopAxis(axis);

                remotePrintf(
                    "%s detenido por final de carrera "
                    "en HOME %.1f grados.\n",
                    axisName(axis),
                    home
                );

                return;
            }

            enterFault(
                softwareLimitFaultForAxis(axis),
                "%s activo fuera de la zona HOME "
                "(distancia %.2f grados)",
                axisName(axis),
                distanceFromHome
            );

            return;
        }

        if (state.limitReleaseAllowed)
        {
            const double releaseTravel =
                fabs(
                    countsToAntennaDegrees(
                        current -
                        state.limitReleaseStartCount
                    )
                );

            if (
                now -
                    state.limitReleaseStartMs <=
                    HOMING_RELEASE_TIMEOUT_MS &&
                releaseTravel <=
                    HOMING_RELEASE_MAX_DEGREES
            )
            {
                // Se permite seguir hasta liberar el contacto.
            }
            else
            {
                enterFault(
                    stuckFaultForAxis(axis),
                    "%s no libero el final al alejarse",
                    axisName(axis)
                );

                return;
            }
        }
        else
        {
            enterFault(
                stuckFaultForAxis(axis),
                "%s se activo inesperadamente "
                "mientras se alejaba",
                axisName(axis)
            );

            return;
        }
    }
    else
    {
        state.limitReleaseAllowed =
            false;
    }

    const int64_t activity =
        llabs(
            current -
            state.lastActivityCount
        );

    if (
        activity >=
        MIN_ENCODER_ACTIVITY_COUNTS
    )
    {
        state.lastActivityCount =
            current;

        state.lastActivityMs =
            now;
    }

    if (state.mode == AxisMode::JOG)
    {
        rampAxisTowardPwm(
            axis,
            state.maximumPwm
        );
    }
    else
    {
        const int64_t remaining =
            state.targetCount -
            current;

        const int64_t travelled =
            current -
            state.startCount;

        int64_t tolerance =
            llabs(
                antennaDegreesToCounts(
                    POSITION_TOLERANCE_DEGREES
                )
            );

        if (tolerance < 1)
        {
            tolerance = 1;
        }

        const bool reached =
            (
                state.direction > 0 &&
                remaining <= tolerance
            ) ||
            (
                state.direction < 0 &&
                remaining >= -tolerance
            );

        if (reached)
        {
            stopAxis(axis);

            remotePrintf(
                "%s alcanzo %.3f grados. "
                "Actual: %.3f\n",
                axisName(axis),
                state.targetDegrees,
                currentDegrees
            );

            return;
        }

        if (
            state.currentPwm >=
                MIN_POSITION_PWM &&
            static_cast<int64_t>(
                state.direction
            ) *
            travelled <
                -16
        )
        {
            enterFault(
                directionFaultForAxis(axis),
                "Encoder de %s cuenta "
                "en sentido contrario",
                axisName(axis)
            );

            return;
        }

        const double remainingDegrees =
            fabs(
                countsToAntennaDegrees(
                    remaining
                )
            );

        rampAxisTowardPwm(
            axis,
            calculateDesiredPositionPwm(
                state,
                remainingDegrees
            )
        );
    }

    if (
        state.currentPwm >=
            MIN_POSITION_PWM &&
        now -
            state.movementStartMs >=
            MOTOR_STARTUP_GRACE_MS &&
        now -
            state.lastActivityMs >
            ENCODER_STALL_TIMEOUT_MS
    )
    {
        enterFault(
            stallFaultForAxis(axis),
            "%s sin progreso del encoder",
            axisName(axis)
        );
    }
}

bool startManualJog(
    uint8_t axis,
    int8_t direction,
    uint8_t maximumPwm
)
{
    stopAllMotion(true);

    if (
        trackRequested &&
        !setTrackRequestedPersisted(
            false,
            false
        )
    )
    {
        return false;
    }

    AxisState& state =
        axes[axis];

    const int64_t current =
        readEncoderCount(axis);

    state.mode =
        AxisMode::JOG;

    state.profile =
        MotionProfile::MANUAL_JOG;

    state.startCount =
        current;

    state.lastActivityCount =
        current;

    state.movementStartMs =
        millis();

    state.lastActivityMs =
        millis();

    state.lastRampMs =
        millis();

    state.maximumPwm =
        maximumPwm;

    configureAxisDirection(
        axis,
        direction
    );

    configureLimitReleaseAllowance(
        axis,
        direction
    );

    activePurpose =
        MovementPurpose::MANUAL_JOG;

    activeMotionProfile =
        MotionProfile::MANUAL_JOG;

    trackerState =
        TrackerState::MANUAL;

    return true;
}

void completeMovementPurpose()
{
    if (
        !allAxesIdle() ||
        activePurpose ==
            MovementPurpose::NONE
    )
    {
        return;
    }

    const MovementPurpose completed =
        activePurpose;

    activePurpose =
        MovementPurpose::NONE;

    if (
        completed ==
        MovementPurpose::AUTO_SUN
    )
    {
        trackerState =
            TrackerState::TRACKING;
    }
    else if (
        completed ==
        MovementPurpose::AUTO_PARK
    )
    {
        trackerState =
            TrackerState::PARKED;
    }
    else
    {
        trackerState =
            TrackerState::MANUAL;
    }
}

// ============================================================
// FAULT Y REINICIO AUTOMATICO
// ============================================================

uint32_t currentFaultRestartDelayMs()
{
    const uint32_t attempt =
        consecutiveFaultRestarts == 0
            ? 1
            : consecutiveFaultRestarts;

    const size_t index =
        attempt <=
            FAULT_RESTART_DELAY_COUNT
                ? attempt - 1
                : FAULT_RESTART_DELAY_COUNT - 1;

    return
        FAULT_RESTART_DELAYS_MS[index];
}

void enterFault(
    FaultCode code,
    const char* format,
    ...
)
{
    if (faultActive)
    {
        return;
    }

    stopAllMotion(true);

    positionCalibrated = false;
    homingPhase = HomingPhase::IDLE;

    faultActive = true;
    activeFaultCode = code;

    va_list arguments;
    va_start(arguments, format);

    vsnprintf(
        activeFaultMessage,
        sizeof(activeFaultMessage),
        format,
        arguments
    );

    va_end(arguments);

    rtcFaultMagic =
        FAULT_RTC_MAGIC;

    if (
        consecutiveFaultRestarts <
        UINT32_MAX
    )
    {
        ++consecutiveFaultRestarts;
    }

    rtcLastFaultCode =
        static_cast<uint8_t>(code);

    snprintf(
        rtcLastFaultMessage,
        sizeof(rtcLastFaultMessage),
        "%s",
        activeFaultMessage
    );

    const uint32_t delayMs =
        currentFaultRestartDelayMs();

    faultRestartAtMs =
        millis() +
        delayMs;

    trackerState =
        TrackerState::FAULT_RESTART_PENDING;

    remotePrintf(
        "\nFAULT %s: %s\n",
        faultCodeName(code),
        activeFaultMessage
    );

    remotePrintf(
        "Motores detenidos. Reinicio automatico "
        "en %lu segundos. TRACK NVS permanece %s.\n",
        static_cast<unsigned long>(
            delayMs /
            1000U
        ),
        trackRequested
            ? "ON"
            : "OFF"
    );
}

void serviceFaultRestart()
{
    if (!faultActive)
    {
        return;
    }

    const int32_t remaining =
        static_cast<int32_t>(
            faultRestartAtMs -
            millis()
        );

    if (remaining <= 0)
    {
        remotePrintln(
            "Reiniciando ESP32 para intentar "
            "recuperacion..."
        );

        delay(100);

        ESP.restart();
    }
}

bool previousFaultBlocksHomingMotion(
    uint8_t axis
)
{
    if (
        rtcFaultMagic != FAULT_RTC_MAGIC ||
        consecutiveFaultRestarts == 0
    )
    {
        return false;
    }

    const FaultCode previous =
        static_cast<FaultCode>(
            rtcLastFaultCode
        );

    if (
        previous ==
            stuckFaultForAxis(axis) &&
        limitActive(axis)
    )
    {
        return true;
    }

    if (
        previous ==
            notFoundFaultForAxis(axis) &&
        consecutiveFaultRestarts >= 2 &&
        !limitActive(axis)
    )
    {
        return true;
    }

    if (
        consecutiveFaultRestarts >= 3 &&
        (
            previous ==
                stallFaultForAxis(axis) ||
            previous ==
                directionFaultForAxis(axis)
        )
    )
    {
        return true;
    }

    return false;
}

// ============================================================
// HOMING NO BLOQUEANTE
// ============================================================

uint32_t homingSeekTimeoutMs(uint8_t axis)
{
    return
        axis == AZIMUTH_AXIS
            ? HOMING_AZIMUTH_TIMEOUT_MS
            : HOMING_ELEVATION_TIMEOUT_MS;
}

double homingMaxTravelDegrees(uint8_t axis)
{
    return
        axis == AZIMUTH_AXIS
            ? HOMING_AZIMUTH_MAX_TRAVEL_DEGREES
            : HOMING_ELEVATION_MAX_TRAVEL_DEGREES;
}

void startHomingMotor(
    uint8_t axis,
    int8_t direction,
    uint8_t pwm
)
{
    AxisState& state =
        axes[axis];

    const int64_t current =
        readEncoderCount(axis);

    state.mode =
        AxisMode::HOMING;

    state.profile =
        MotionProfile::MANUAL_JOG;

    state.startCount =
        current;

    state.lastActivityCount =
        current;

    state.movementStartMs =
        millis();

    state.lastActivityMs =
        millis();

    state.lastRampMs =
        millis();

    state.maximumPwm =
        pwm;

    configureAxisDirection(
        axis,
        direction
    );

    state.limitReleaseAllowed =
        false;

    writeAxisPwm(
        axis,
        pwm
    );

    homingPhaseStartCount =
        current;

    homingPhaseStartMs =
        millis();
}

bool serviceHomingMotorSafety(
    uint8_t axis,
    double maxTravelDegrees,
    uint32_t timeoutMs
)
{
    AxisState& state =
        axes[axis];

    const uint32_t now =
        millis();

    const int64_t current =
        readEncoderCount(axis);

    if (
        llabs(
            current -
            state.lastActivityCount
        ) >=
        MIN_ENCODER_ACTIVITY_COUNTS
    )
    {
        state.lastActivityCount =
            current;

        state.lastActivityMs =
            now;
    }

    const int64_t travelled =
        current -
        state.startCount;

    if (
        state.currentPwm >=
            MIN_POSITION_PWM &&
        static_cast<int64_t>(
            state.direction
        ) *
        travelled <
            -16
    )
    {
        enterFault(
            directionFaultForAxis(axis),
            "Encoder de %s cuenta al reves "
            "durante homing",
            axisName(axis)
        );

        return false;
    }

    if (
        state.currentPwm >=
            MIN_POSITION_PWM &&
        now -
            state.movementStartMs >=
            MOTOR_STARTUP_GRACE_MS &&
        now -
            state.lastActivityMs >
            ENCODER_STALL_TIMEOUT_MS
    )
    {
        enterFault(
            stallFaultForAxis(axis),
            "%s sin encoder durante homing",
            axisName(axis)
        );

        return false;
    }

    if (
        fabs(
            countsToAntennaDegrees(
                current -
                homingPhaseStartCount
            )
        ) >
        maxTravelDegrees
    )
    {
        enterFault(
            notFoundFaultForAxis(axis),
            "%s excedio el recorrido maximo "
            "de homing",
            axisName(axis)
        );

        return false;
    }

    if (
        now -
        homingPhaseStartMs >
        timeoutMs
    )
    {
        enterFault(
            FaultCode::HOMING_TIMEOUT,
            "Timeout de homing en %s",
            axisName(axis)
        );

        return false;
    }

    return true;
}

void startHoming()
{
    stopAllMotion(true);

    faultActive = false;

    activeFaultCode =
        FaultCode::NONE;

    activeFaultMessage[0] =
        '\0';

    positionCalibrated = false;

    daylightStateKnown = false;
    lastSolarPositionValid = false;

    homingOrderIndex = 0;
    homingAxis = ELEVATION_AXIS;

    homingPhase =
        HomingPhase::START_AXIS;

    trackerState =
        TrackerState::HOMING;

    remotePrintln(
        "Iniciando calibracion fisica: "
        "ELEVACION y luego AZIMUT."
    );
}

void setAxisHomeReference(uint8_t axis)
{
    const double home =
        axis == AZIMUTH_AXIS
            ? HOME_AZIMUTH_DEGREES
            : HOME_ELEVATION_DEGREES;

    setEncoderCount(
        axis,
        antennaDegreesToCounts(
            home
        )
    );

    remotePrintf(
        "%s calibrado en %.1f grados.\n",
        axisName(axis),
        home
    );
}

void serviceHoming()
{
    if (
        faultActive ||
        homingPhase ==
            HomingPhase::IDLE
    )
    {
        return;
    }

    trackerState =
        TrackerState::HOMING;

    const uint8_t axis =
        homingAxis;

    const int8_t toward =
        HOMING_TOWARD_LIMIT_DIRECTION[axis];

    const int8_t away =
        -toward;

    const uint32_t now =
        millis();

    switch (homingPhase)
    {
        case HomingPhase::START_AXIS:
        {
            stopAxis(axis);

            if (
                previousFaultBlocksHomingMotion(
                    axis
                )
            )
            {
                const FaultCode previous =
                    static_cast<FaultCode>(
                        rtcLastFaultCode
                    );

                enterFault(
                    previous,
                    "%s: condicion anterior continua; "
                    "eje bloqueado por seguridad",
                    axisName(axis)
                );

                return;
            }

            remotePrintf(
                "Homing de %s. Final actual: %s\n",
                axisName(axis),
                limitActive(axis)
                    ? "ACTIVO"
                    : "LIBRE"
            );

            if (limitActive(axis))
            {
                startHomingMotor(
                    axis,
                    away,
                    HOMING_RELEASE_PWM
                );

                homingPhase =
                    HomingPhase::RELEASE_SWITCH;
            }
            else
            {
                startHomingMotor(
                    axis,
                    toward,
                    HOMING_FAST_PWM
                );

                homingPhase =
                    HomingPhase::SEEK_FAST;
            }

            break;
        }

        case HomingPhase::RELEASE_SWITCH:
        {
            if (
                !serviceHomingMotorSafety(
                    axis,
                    HOMING_RELEASE_MAX_DEGREES,
                    HOMING_RELEASE_TIMEOUT_MS
                )
            )
            {
                return;
            }

            if (!limitActive(axis))
            {
                homingReleaseCount =
                    readEncoderCount(axis);

                homingPhaseStartCount =
                    homingReleaseCount;

                homingPhaseStartMs =
                    now;

                homingPhase =
                    HomingPhase::RELEASE_CLEARANCE;
            }

            break;
        }

        case HomingPhase::RELEASE_CLEARANCE:
        {
            if (
                !serviceHomingMotorSafety(
                    axis,
                    HOMING_RELEASE_MAX_DEGREES,
                    HOMING_RELEASE_TIMEOUT_MS
                )
            )
            {
                return;
            }

            if (
                fabs(
                    countsToAntennaDegrees(
                        readEncoderCount(axis) -
                        homingReleaseCount
                    )
                ) >=
                HOMING_BACKOFF_DEGREES
            )
            {
                stopAxis(axis);

                startHomingMotor(
                    axis,
                    toward,
                    HOMING_FAST_PWM
                );

                homingPhase =
                    HomingPhase::SEEK_FAST;
            }

            break;
        }

        case HomingPhase::SEEK_FAST:
        {
            if (
                !serviceHomingMotorSafety(
                    axis,
                    homingMaxTravelDegrees(axis),
                    homingSeekTimeoutMs(axis)
                )
            )
            {
                return;
            }

            if (limitActive(axis))
            {
                stopAxis(axis);

                homingPhaseStartMs =
                    now;

                homingPhase =
                    HomingPhase::WAIT_FAST_HIT;
            }

            break;
        }

        case HomingPhase::WAIT_FAST_HIT:
        {
            if (
                now -
                homingPhaseStartMs >=
                HOMING_SWITCH_SETTLE_MS
            )
            {
                if (!limitActive(axis))
                {
                    startHomingMotor(
                        axis,
                        toward,
                        HOMING_FAST_PWM
                    );

                    homingPhase =
                        HomingPhase::SEEK_FAST;
                }
                else
                {
                    startHomingMotor(
                        axis,
                        away,
                        HOMING_RELEASE_PWM
                    );

                    homingPhase =
                        HomingPhase::BACKOFF_RELEASE;
                }
            }

            break;
        }

        case HomingPhase::BACKOFF_RELEASE:
        {
            if (
                !serviceHomingMotorSafety(
                    axis,
                    HOMING_RELEASE_MAX_DEGREES,
                    HOMING_RELEASE_TIMEOUT_MS
                )
            )
            {
                return;
            }

            if (!limitActive(axis))
            {
                homingReleaseCount =
                    readEncoderCount(axis);

                homingPhaseStartCount =
                    homingReleaseCount;

                homingPhaseStartMs =
                    now;

                homingPhase =
                    HomingPhase::BACKOFF_CLEARANCE;
            }

            break;
        }

        case HomingPhase::BACKOFF_CLEARANCE:
        {
            if (
                !serviceHomingMotorSafety(
                    axis,
                    HOMING_RELEASE_MAX_DEGREES,
                    HOMING_RELEASE_TIMEOUT_MS
                )
            )
            {
                return;
            }

            if (
                fabs(
                    countsToAntennaDegrees(
                        readEncoderCount(axis) -
                        homingReleaseCount
                    )
                ) >=
                HOMING_BACKOFF_DEGREES
            )
            {
                stopAxis(axis);

                homingPhaseStartMs =
                    now;

                homingPhase =
                    HomingPhase::WAIT_BEFORE_SLOW;
            }

            break;
        }

        case HomingPhase::WAIT_BEFORE_SLOW:
        {
            if (
                now -
                homingPhaseStartMs >=
                HOMING_SWITCH_SETTLE_MS
            )
            {
                startHomingMotor(
                    axis,
                    toward,
                    HOMING_SLOW_PWM
                );

                homingPhase =
                    HomingPhase::SEEK_SLOW;
            }

            break;
        }

        case HomingPhase::SEEK_SLOW:
        {
            if (
                !serviceHomingMotorSafety(
                    axis,
                    HOMING_BACKOFF_DEGREES + 2.0,
                    HOMING_RELEASE_TIMEOUT_MS
                )
            )
            {
                return;
            }

            if (limitActive(axis))
            {
                stopAxis(axis);

                setAxisHomeReference(axis);

                homingPhaseStartMs =
                    now;

                homingPhase =
                    HomingPhase::AXIS_COMPLETE;
            }

            break;
        }

        case HomingPhase::AXIS_COMPLETE:
        {
            if (
                now -
                homingPhaseStartMs <
                HOMING_SWITCH_SETTLE_MS
            )
            {
                return;
            }

            ++homingOrderIndex;

            if (homingOrderIndex == 1)
            {
                homingAxis =
                    AZIMUTH_AXIS;

                homingPhase =
                    HomingPhase::START_AXIS;
            }
            else
            {
                homingPhase =
                    HomingPhase::COMPLETE;
            }

            break;
        }

        case HomingPhase::COMPLETE:
        {
            stopAllMotion(true);

            positionCalibrated = true;

            homingPhase =
                HomingPhase::IDLE;

            consecutiveFaultRestarts = 0;

            faultActive = false;

            activeFaultCode =
                FaultCode::NONE;

            forceSolarUpdate = true;

            remotePrintln(
                "Calibracion completa: "
                "AZ=0 grados, EL=90 grados."
            );

            refreshTrackerState();

            break;
        }

        default:
            break;
    }
}

// ============================================================
// ESTADO
// ============================================================

void refreshTrackerState()
{
    if (faultActive)
    {
        trackerState =
            TrackerState::FAULT_RESTART_PENDING;

        return;
    }

    if (
        homingPhase !=
        HomingPhase::IDLE
    )
    {
        trackerState =
            TrackerState::HOMING;

        return;
    }

    if (anyAxisMoving())
    {
        return;
    }

    if (!positionCalibrated)
    {
        trackerState =
            TrackerState::STARTUP;

        return;
    }

    if (
        !rtcValid ||
        !rtcBackupConfigured
    )
    {
        trackerState =
            TrackerState::RTC_INVALID;

        return;
    }

    if (!locationValid)
    {
        trackerState =
            TrackerState::LOCATION_INVALID;

        return;
    }

    if (trackRequested)
    {
        trackerState =
            daylightStateKnown &&
            daylightActive
                ? TrackerState::TRACKING
                : TrackerState::PARKED;
    }
    else
    {
        trackerState =
            TrackerState::READY_HOME;
    }
}

// ============================================================
// CALCULO SOLAR
// ============================================================

uint16_t dayOfYear(
    const RtcDateTime& dt
)
{
    uint16_t result =
        dt.day;

    for (
        uint8_t month = 1;
        month < dt.month;
        ++month
    )
    {
        result +=
            daysInMonth(
                dt.year,
                month
            );
    }

    return result;
}

double atmosphericRefractionCorrection(
    double elevation
)
{
    if (elevation > 85.0)
    {
        return 0.0;
    }

    const double radians =
        degreesToRadians(
            elevation
        );

    if (elevation > 5.0)
    {
        const double t =
            tan(radians);

        return
            (
                58.1 / t -
                0.07 /
                    (
                        t *
                        t *
                        t
                    ) +
                0.000086 /
                    (
                        t *
                        t *
                        t *
                        t *
                        t
                    )
            ) /
            3600.0;
    }

    if (elevation > -0.575)
    {
        const double h =
            elevation;

        return
            (
                1735.0 -
                518.2 * h +
                103.4 * h * h -
                12.79 * h * h * h +
                0.711 * h * h * h * h
            ) /
            3600.0;
    }

    return
        (
            -20.774 /
            tan(radians)
        ) /
        3600.0;
}

bool calculateSolarPosition(
    const RtcDateTime& utc,
    double latitude,
    double longitude,
    SolarPosition& result
)
{
    if (
        !validateDateTime(utc) ||
        latitude < -90.0 ||
        latitude > 90.0 ||
        longitude < -180.0 ||
        longitude > 180.0
    )
    {
        return false;
    }

    const uint16_t yearDays =
        isLeapYear(utc.year)
            ? 366U
            : 365U;

    const uint16_t ordinal =
        dayOfYear(utc);

    const double decimalHour =
        utc.hour +
        utc.minute / 60.0 +
        utc.second / 3600.0;

    const double gamma =
        2.0 *
        PI_D /
        yearDays *
        (
            ordinal -
            1.0 +
            (
                decimalHour -
                12.0
            ) /
            24.0
        );

    const double equationOfTime =
        229.18 *
        (
            0.000075 +
            0.001868 *
                cos(gamma) -
            0.032077 *
                sin(gamma) -
            0.014615 *
                cos(
                    2.0 *
                    gamma
                ) -
            0.040849 *
                sin(
                    2.0 *
                    gamma
                )
        );

    const double declination =
        0.006918 -
        0.399912 *
            cos(gamma) +
        0.070257 *
            sin(gamma) -
        0.006758 *
            cos(
                2.0 *
                gamma
            ) +
        0.000907 *
            sin(
                2.0 *
                gamma
            ) -
        0.002697 *
            cos(
                3.0 *
                gamma
            ) +
        0.001480 *
            sin(
                3.0 *
                gamma
            );

    double trueSolarTime =
        decimalHour *
            60.0 +
        equationOfTime +
        4.0 *
            longitude;

    trueSolarTime =
        fmod(
            trueSolarTime,
            1440.0
        );

    if (trueSolarTime < 0.0)
    {
        trueSolarTime += 1440.0;
    }

    double hourAngleDegrees =
        trueSolarTime /
            4.0 -
        180.0;

    if (hourAngleDegrees < -180.0)
    {
        hourAngleDegrees += 360.0;
    }

    const double latRad =
        degreesToRadians(
            latitude
        );

    const double haRad =
        degreesToRadians(
            hourAngleDegrees
        );

    double cosineZenith =
        sin(latRad) *
            sin(declination) +
        cos(latRad) *
            cos(declination) *
            cos(haRad);

    cosineZenith =
        clampDouble(
            cosineZenith,
            -1.0,
            1.0
        );

    const double geometricElevation =
        90.0 -
        radiansToDegrees(
            acos(cosineZenith)
        );

    double azimuth =
        radiansToDegrees(
            atan2(
                sin(haRad),
                cos(haRad) *
                    sin(latRad) -
                tan(declination) *
                    cos(latRad)
            )
        ) +
        180.0;

    azimuth =
        normalizeDegrees360(
            azimuth
        );

    result.azimuthDegrees =
        azimuth;

    result.geometricElevationDegrees =
        geometricElevation;

    result.apparentElevationDegrees =
        geometricElevation +
        atmosphericRefractionCorrection(
            geometricElevation
        );

    result.declinationDegrees =
        radiansToDegrees(
            declination
        );

    result.equationOfTimeMinutes =
        equationOfTime;

    result.hourAngleDegrees =
        hourAngleDegrees;

    return true;
}

bool getCurrentSolarPosition(
    SolarPosition& solar,
    RtcDateTime* utcOut = nullptr
)
{
    if (
        !rtcValid ||
        !locationValid
    )
    {
        return false;
    }

    RtcDateTime utc;

    if (!rtcReadDateTime(utc))
    {
        rtcValid = false;

        refreshTrackerState();

        return false;
    }

    if (
        !calculateSolarPosition(
            utc,
            latitudeDegrees,
            longitudeDegrees,
            solar
        )
    )
    {
        return false;
    }

    if (utcOut)
    {
        *utcOut =
            utc;
    }

    return true;
}

void updateDaylightState(double elevation)
{
    if (!daylightStateKnown)
    {
        daylightActive =
            elevation > 0.0;

        daylightStateKnown =
            true;

        return;
    }

    if (
        daylightActive &&
        elevation <=
            NIGHT_ENTER_ELEVATION_DEGREES
    )
    {
        daylightActive =
            false;
    }
    else if (
        !daylightActive &&
        elevation >=
            DAY_ENTER_ELEVATION_DEGREES
    )
    {
        daylightActive =
            true;
    }
}

void printSolarPosition()
{
    SolarPosition solar;
    RtcDateTime utc;

    if (
        !getCurrentSolarPosition(
            solar,
            &utc
        )
    )
    {
        remotePrintln(
            "No se puede calcular el Sol. "
            "Revise RTC y ubicacion."
        );

        return;
    }

    remotePrintf(
        "UTC %04u-%02u-%02u "
        "%02u:%02u:%02u | "
        "AZ=%.4f | EL geom=%.4f | "
        "EL aparente=%.4f\n",
        utc.year,
        utc.month,
        utc.day,
        utc.hour,
        utc.minute,
        utc.second,
        solar.azimuthDegrees,
        solar.geometricElevationDegrees,
        solar.apparentElevationDegrees
    );
}

// ============================================================
// TRACKING AUTOMATICO
// ============================================================

void serviceAutomaticTracking()
{
    if (
        !trackingOperational() ||
        anyAxisMoving()
    )
    {
        return;
    }

    const uint32_t now =
        millis();

    if (
        !forceSolarUpdate &&
        now -
            lastSolarUpdateMs <
            SOLAR_UPDATE_INTERVAL_MS
    )
    {
        return;
    }

    forceSolarUpdate =
        false;

    lastSolarUpdateMs =
        now;

    SolarPosition solar;

    if (!getCurrentSolarPosition(solar))
    {
        remotePrintln(
            "Seguimiento suspendido: "
            "no se pudo calcular el Sol."
        );

        return;
    }

    lastSolarPosition =
        solar;

    lastSolarPositionValid =
        true;

    updateDaylightState(
        solar.apparentElevationDegrees
    );

    if (!daylightActive)
    {
        const double az =
            currentAxisDegrees(
                AZIMUTH_AXIS
            );

        const double el =
            currentAxisDegrees(
                ELEVATION_AXIS
            );

        if (
            fabs(
                az -
                PARK_AZIMUTH_DEGREES
            ) >
                TRACKING_DEADBAND_DEGREES ||
            fabs(
                el -
                PARK_ELEVATION_DEGREES
            ) >
                TRACKING_DEADBAND_DEGREES
        )
        {
            remotePrintln(
                "Noche: regresando a PARK 0/0."
            );

            startCoordinatedAbsoluteMove(
                PARK_AZIMUTH_DEGREES,
                PARK_ELEVATION_DEGREES,
                MovementPurpose::AUTO_PARK,
                MotionProfile::PARK
            );
        }
        else
        {
            trackerState =
                TrackerState::PARKED;
        }

        return;
    }

    const double targetAz =
        clampDouble(
            solar.azimuthDegrees,
            AZIMUTH_MIN_DEGREES,
            AZIMUTH_MAX_DEGREES
        );

    const double targetEl =
        clampDouble(
            solar.apparentElevationDegrees,
            ELEVATION_MIN_DEGREES,
            ELEVATION_MAX_DEGREES
        );

    const double azError =
        fabs(
            targetAz -
            currentAxisDegrees(
                AZIMUTH_AXIS
            )
        );

    const double elError =
        fabs(
            targetEl -
            currentAxisDegrees(
                ELEVATION_AXIS
            )
        );

    if (
        azError <=
            TRACKING_DEADBAND_DEGREES &&
        elError <=
            TRACKING_DEADBAND_DEGREES
    )
    {
        trackerState =
            TrackerState::TRACKING;

        return;
    }

    const double maxError =
        azError > elError
            ? azError
            : elError;

    const MotionProfile profile =
        maxError >
            SLEW_THRESHOLD_DEGREES
                ? MotionProfile::SLEW
                : MotionProfile::TRACK;

    remotePrintf(
        "Objetivo solar: AZ=%.3f EL=%.3f | "
        "error AZ=%.3f EL=%.3f\n",
        targetAz,
        targetEl,
        azError,
        elError
    );

    startCoordinatedAbsoluteMove(
        targetAz,
        targetEl,
        MovementPurpose::AUTO_SUN,
        profile
    );
}

// ============================================================
// ESTADO Y CONSULTAS
// ============================================================

void printPosition()
{
    remotePrintf(
        "Posicion calibrada: %s | "
        "AZ=%.4f (%lld) | "
        "EL=%.4f (%lld)\n",
        positionCalibrated
            ? "SI"
            : "NO",
        currentAxisDegrees(
            AZIMUTH_AXIS
        ),
        static_cast<long long>(
            readEncoderCount(
                AZIMUTH_AXIS
            )
        ),
        currentAxisDegrees(
            ELEVATION_AXIS
        ),
        static_cast<long long>(
            readEncoderCount(
                ELEVATION_AXIS
            )
        )
    );
}

void printLocation()
{
    remotePrintf(
        "Ubicacion valida: %s | "
        "lat=%.8f lon=%.8f alt=%.2f m\n",
        locationValid
            ? "SI"
            : "NO",
        latitudeDegrees,
        longitudeDegrees,
        altitudeMeters
    );
}

void printLimits()
{
    remotePrintf(
        "Final Motor 1 / AZ: %s | "
        "Final Motor 2 / EL: %s\n",
        limitActive(
            AZIMUTH_AXIS
        )
            ? "ACTIVO/HIGH"
            : "LIBRE/LOW",
        limitActive(
            ELEVATION_AXIS
        )
            ? "ACTIVO/HIGH"
            : "LIBRE/LOW"
    );
}

void printStatus()
{
    const bool persistedTrack =
        lastCommittedRecordValid &&
        lastCommittedRecord.trackRequested != 0;

    const bool persistedMatches =
        lastCommittedRecordValid &&
        persistedTrack ==
            trackRequested;

    const uint32_t remainingMs =
        faultActive &&
        static_cast<int32_t>(
            faultRestartAtMs -
            millis()
        ) >
        0
            ? faultRestartAtMs -
                millis()
            : 0;

    remotePrintln(
        "\n========== ESTADO DEL TRACKER =========="
    );

    remotePrintf(
        "Estado: %s\n",
        trackerStateName(
            trackerState
        )
    );

    remotePrintf(
        "Homing: %s | eje=%s\n",
        homingPhaseName(
            homingPhase
        ),
        axisName(
            homingAxis
        )
    );

    remotePrintf(
        "Posicion calibrada: %s\n",
        positionCalibrated
            ? "SI"
            : "NO"
    );

    remotePrintf(
        "TRACK solicitado/NVS: %s\n",
        trackRequested
            ? "ON"
            : "OFF"
    );

    remotePrintf(
        "TRACK guardado verificado: %s\n",
        lastCommittedRecordValid
            ? (
                persistedTrack
                    ? "ON"
                    : "OFF"
            )
            : "NO DISPONIBLE"
    );

    remotePrintf(
        "TRACK coincide con NVS: %s\n",
        persistedMatches
            ? "SI"
            : "NO"
    );

    remotePrintf(
        "TRACK operativo: %s\n",
        trackingOperational()
            ? "ON"
            : (
                trackRequested
                    ? "ESPERANDO CONDICIONES"
                    : "OFF"
            )
    );

    printLimits();

    remotePrintf(
        "FAULT activo: %s\n",
        faultActive
            ? "SI"
            : "NO"
    );

    if (faultActive)
    {
        remotePrintf(
            "FAULT actual: %s - %s\n",
            faultCodeName(
                activeFaultCode
            ),
            activeFaultMessage
        );

        remotePrintf(
            "Reinicio en: %.1f s\n",
            remainingMs /
            1000.0
        );
    }

    remotePrintf(
        "Ultimo FAULT retenido: %s\n",
        faultCodeName(
            static_cast<FaultCode>(
                rtcLastFaultCode
            )
        )
    );

    if (
        rtcLastFaultMessage[0] !=
        '\0'
    )
    {
        remotePrintf(
            "Detalle ultimo FAULT: %s\n",
            rtcLastFaultMessage
        );
    }

    remotePrintf(
        "Reinicios consecutivos por FAULT: %lu\n",
        static_cast<unsigned long>(
            consecutiveFaultRestarts
        )
    );

    remotePrintf(
        "WiFi: %s\n",
        WiFi.status() ==
            WL_CONNECTED
                ? "CONECTADO"
                : "DESCONECTADO"
    );

    remotePrintf(
        "RTC detectado: %s | "
        "hora valida: %s | "
        "respaldo: %s\n",
        rtcDetected
            ? "SI"
            : "NO",
        rtcValid
            ? "SI"
            : "NO",
        rtcBackupConfigured
            ? "SI"
            : "NO"
    );

    printLocation();
    printPosition();

    if (lastSolarPositionValid)
    {
        remotePrintf(
            "Ultimo Sol: AZ=%.4f EL=%.4f\n",
            lastSolarPosition.azimuthDegrees,
            lastSolarPosition.apparentElevationDegrees
        );
    }

    remotePrintln(
        "========================================\n"
    );
}

// ============================================================
// COMANDOS
// ============================================================

void showHelp()
{
    remotePrintln(
        "\n================ COMANDOS ================"
    );

    remotePrintln(
        "SETUTC AAAA-MM-DD HH:MM:SS"
    );

    remotePrintln(
        "TIME | RTCSTATUS"
    );

    remotePrintln(
        "SETLOC latitud longitud altitud_m"
    );

    remotePrintln(
        "LOCATION | SUN | STATUS | POSITION | LIMITS"
    );

    remotePrintln(
        "TRACK ON | TRACK OFF"
    );

    remotePrintln(
        "CALIBRATE | HOME"
    );

    remotePrintln(
        "GOTO azimut elevacion | PARK"
    );

    remotePrintln(
        "F1 [PWM] | B1 [PWM] | F2 [PWM] | B2 [PWM]"
    );

    remotePrintln(
        "S | S1 | S2 | N1 | N2"
    );

    remotePrintln(
        "RESTART | H"
    );

    remotePrintln(
        "==========================================\n"
    );
}

void processCommand(char* command)
{
    while (
        *command &&
        isspace(
            static_cast<unsigned char>(
                *command
            )
        )
    )
    {
        ++command;
    }

    if (!*command)
    {
        return;
    }

    for (
        char* pointer = command;
        *pointer;
        ++pointer
    )
    {
        *pointer =
            static_cast<char>(
                toupper(
                    static_cast<unsigned char>(
                        *pointer
                    )
                )
            );
    }

    char* savePointer = nullptr;

    char* name =
        strtok_r(
            command,
            " \t",
            &savePointer
        );

    if (!name)
    {
        return;
    }

    if (
        strcmp(
            name,
            "H"
        ) == 0
    )
    {
        showHelp();
    }
    else if (
        strcmp(
            name,
            "STATUS"
        ) == 0
    )
    {
        printStatus();
    }
    else if (
        strcmp(
            name,
            "POSITION"
        ) == 0
    )
    {
        printPosition();
    }
    else if (
        strcmp(
            name,
            "LIMITS"
        ) == 0
    )
    {
        printLimits();
    }
    else if (
        strcmp(
            name,
            "LOCATION"
        ) == 0
    )
    {
        printLocation();
    }
    else if (
        strcmp(
            name,
            "TIME"
        ) == 0
    )
    {
        printRtcTime();
    }
    else if (
        strcmp(
            name,
            "RTCSTATUS"
        ) == 0
    )
    {
        printRtcStatus();
    }
    else if (
        strcmp(
            name,
            "SUN"
        ) == 0
    )
    {
        printSolarPosition();
    }
    else if (
        strcmp(
            name,
            "SETUTC"
        ) == 0
    )
    {
        char* dateText =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        char* timeText =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        if (
            !dateText ||
            !timeText
        )
        {
            remotePrintln(
                "Uso: SETUTC "
                "AAAA-MM-DD HH:MM:SS"
            );

            return;
        }

        unsigned int year;
        unsigned int month;
        unsigned int day;

        unsigned int hour;
        unsigned int minute;
        unsigned int second;

        if (
            sscanf(
                dateText,
                "%u-%u-%u",
                &year,
                &month,
                &day
            ) != 3 ||
            sscanf(
                timeText,
                "%u:%u:%u",
                &hour,
                &minute,
                &second
            ) != 3
        )
        {
            remotePrintln(
                "Formato UTC invalido."
            );

            return;
        }

        RtcDateTime dt;

        dt.year =
            year;

        dt.month =
            month;

        dt.day =
            day;

        dt.hour =
            hour;

        dt.minute =
            minute;

        dt.second =
            second;

        if (!validateDateTime(dt))
        {
            remotePrintln(
                "Fecha u hora fuera de rango."
            );

            return;
        }

        dt.weekday =
            calculateWeekday(
                dt.year,
                dt.month,
                dt.day
            );

        if (rtcSetDateTimeVerified(dt))
        {
            remotePrintln(
                "UTC guardado y verificado."
            );

            refreshTrackerState();

            forceSolarUpdate = true;

            printRtcTime();
        }
        else
        {
            rtcValid =
                rtcCheckValidity();

            remotePrintln(
                "ERROR configurando RTC."
            );

            printRtcStatus();
        }
    }
    else if (
        strcmp(
            name,
            "SETLOC"
        ) == 0
    )
    {
        char* latText =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        char* lonText =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        char* altText =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        if (
            !latText ||
            !lonText ||
            !altText
        )
        {
            remotePrintln(
                "Uso: SETLOC latitud "
                "longitud altitud_m"
            );

            return;
        }

        const double newLat =
            strtod(
                latText,
                nullptr
            );

        const double newLon =
            strtod(
                lonText,
                nullptr
            );

        const double newAlt =
            strtod(
                altText,
                nullptr
            );

        if (
            !isfinite(newLat) ||
            !isfinite(newLon) ||
            !isfinite(newAlt) ||
            newLat < -90.0 ||
            newLat > 90.0 ||
            newLon < -180.0 ||
            newLon > 180.0 ||
            newAlt < -500.0 ||
            newAlt > 10000.0
        )
        {
            remotePrintln(
                "Ubicacion fuera de rango."
            );

            return;
        }

        const bool oldValid =
            locationValid;

        const double oldLat =
            latitudeDegrees;

        const double oldLon =
            longitudeDegrees;

        const double oldAlt =
            altitudeMeters;

        locationValid = true;

        latitudeDegrees =
            newLat;

        longitudeDegrees =
            newLon;

        altitudeMeters =
            newAlt;

        if (!savePersistentConfig(true))
        {
            locationValid =
                oldValid;

            latitudeDegrees =
                oldLat;

            longitudeDegrees =
                oldLon;

            altitudeMeters =
                oldAlt;

            remotePrintln(
                "ERROR guardando ubicacion."
            );

            return;
        }

        forceSolarUpdate =
            true;

        refreshTrackerState();

        remotePrintln(
            "Ubicacion guardada y verificada."
        );

        printLocation();
    }
    else if (
        strcmp(
            name,
            "TRACK"
        ) == 0
    )
    {
        char* mode =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        if (!mode)
        {
            remotePrintf(
                "TRACK solicitado: %s | "
                "operativo: %s\n",
                trackRequested
                    ? "ON"
                    : "OFF",
                trackingOperational()
                    ? "ON"
                    : "OFF/ESPERA"
            );

            return;
        }

        if (
            strcmp(
                mode,
                "ON"
            ) == 0
        )
        {
            if (
                !setTrackRequestedPersisted(
                    true,
                    true
                )
            )
            {
                return;
            }

            forceSolarUpdate =
                true;

            refreshTrackerState();

            if (!positionCalibrated)
            {
                remotePrintln(
                    "TRACK queda pendiente hasta "
                    "completar homing."
                );
            }
            else if (
                !rtcValid ||
                !rtcBackupConfigured ||
                !locationValid
            )
            {
                remotePrintln(
                    "TRACK guardado ON, pero espera "
                    "RTC/ubicacion validos."
                );
            }
        }
        else if (
            strcmp(
                mode,
                "OFF"
            ) == 0
        )
        {
            if (
                activePurpose ==
                    MovementPurpose::AUTO_SUN ||
                activePurpose ==
                    MovementPurpose::AUTO_PARK
            )
            {
                stopAllMotion(true);
            }

            if (
                !setTrackRequestedPersisted(
                    false,
                    true
                )
            )
            {
                return;
            }

            refreshTrackerState();
        }
        else
        {
            remotePrintln(
                "Uso: TRACK ON o TRACK OFF"
            );
        }
    }
    else if (
        strcmp(
            name,
            "CALIBRATE"
        ) == 0 ||
        strcmp(
            name,
            "HOME"
        ) == 0
    )
    {
        if (faultActive)
        {
            remotePrintln(
                "Hay un FAULT activo; el reinicio "
                "automatico realizara el nuevo intento."
            );

            return;
        }

        startHoming();
    }
    else if (
        strcmp(
            name,
            "GOTO"
        ) == 0
    )
    {
        char* azText =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        char* elText =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        if (
            !azText ||
            !elText
        )
        {
            remotePrintln(
                "Uso: GOTO azimut elevacion"
            );

            return;
        }

        if (
            trackRequested &&
            !setTrackRequestedPersisted(
                false,
                false
            )
        )
        {
            return;
        }

        startCoordinatedAbsoluteMove(
            strtod(
                azText,
                nullptr
            ),
            strtod(
                elText,
                nullptr
            ),
            MovementPurpose::MANUAL_GOTO,
            MotionProfile::MANUAL_GOTO
        );
    }
    else if (
        strcmp(
            name,
            "PARK"
        ) == 0
    )
    {
        if (
            trackRequested &&
            !setTrackRequestedPersisted(
                false,
                false
            )
        )
        {
            return;
        }

        startCoordinatedAbsoluteMove(
            PARK_AZIMUTH_DEGREES,
            PARK_ELEVATION_DEGREES,
            MovementPurpose::MANUAL_GOTO,
            MotionProfile::PARK
        );
    }
    else if (
        strcmp(
            name,
            "F1"
        ) == 0 ||
        strcmp(
            name,
            "B1"
        ) == 0 ||
        strcmp(
            name,
            "F2"
        ) == 0 ||
        strcmp(
            name,
            "B2"
        ) == 0
    )
    {
        if (!positionCalibrated)
        {
            remotePrintln(
                "Primero complete CALIBRATE."
            );

            return;
        }

        const uint8_t axis =
            name[1] == '1'
                ? AZIMUTH_AXIS
                : ELEVATION_AXIS;

        const int8_t direction =
            name[0] == 'F'
                ? 1
                : -1;

        char* pwmText =
            strtok_r(
                nullptr,
                " \t",
                &savePointer
            );

        uint8_t pwm =
            pwmText
                ? limitPwm(
                    strtoul(
                        pwmText,
                        nullptr,
                        10
                    )
                )
                : MANUAL_MAX_PWM;

        if (pwm < MIN_POSITION_PWM)
        {
            pwm =
                MIN_POSITION_PWM;
        }

        if (
            startManualJog(
                axis,
                direction,
                pwm
            )
        )
        {
            remotePrintf(
                "%s manual direccion=%d PWM=%u\n",
                axisName(axis),
                direction,
                pwm
            );
        }
    }
    else if (
        strcmp(
            name,
            "S"
        ) == 0
    )
    {
        stopAllMotion(true);

        if (
            homingPhase !=
            HomingPhase::IDLE
        )
        {
            homingPhase =
                HomingPhase::IDLE;

            positionCalibrated =
                false;
        }

        setTrackRequestedPersisted(
            false,
            false
        );

        refreshTrackerState();

        remotePrintln(
            "Parada total. TRACK OFF guardado."
        );
    }
    else if (
        strcmp(
            name,
            "S1"
        ) == 0 ||
        strcmp(
            name,
            "S2"
        ) == 0
    )
    {
        const uint8_t axis =
            name[1] == '1'
                ? AZIMUTH_AXIS
                : ELEVATION_AXIS;

        stopAxis(axis);

        activePurpose =
            MovementPurpose::NONE;

        if (
            homingPhase !=
            HomingPhase::IDLE
        )
        {
            stopAllMotion(true);

            homingPhase =
                HomingPhase::IDLE;

            positionCalibrated =
                false;
        }

        setTrackRequestedPersisted(
            false,
            false
        );

        refreshTrackerState();

        remotePrintf(
            "%s detenido. TRACK OFF.\n",
            axisName(axis)
        );
    }
    else if (
        strcmp(
            name,
            "N1"
        ) == 0 ||
        strcmp(
            name,
            "N2"
        ) == 0
    )
    {
        const uint8_t axis =
            name[1] == '1'
                ? AZIMUTH_AXIS
                : ELEVATION_AXIS;

        remotePrintf(
            "%s: %lld conteos, %.5f grados\n",
            axisName(axis),
            static_cast<long long>(
                readEncoderCount(axis)
            ),
            currentAxisDegrees(axis)
        );
    }
    else if (
        strcmp(
            name,
            "RESTART"
        ) == 0
    )
    {
        remotePrintln(
            "Reinicio manual solicitado."
        );

        delay(100);

        ESP.restart();
    }
    else
    {
        remotePrintln(
            "Comando desconocido. Use H."
        );
    }
}

// ============================================================
// TELNET
// ============================================================

void serviceTelnet()
{
    if (telnetServer.hasClient())
    {
        WiFiClient newClient =
            telnetServer.available();

        if (
            !telnetClient ||
            !telnetClient.connected()
        )
        {
            telnetClient.stop();

            telnetClient =
                newClient;

            telnetClient.setNoDelay(
                true
            );

            telnetCommandLength = 0;
            telnetBytesToIgnore = 0;
            telnetWasConnected = true;

            telnetClient.println();

            telnetClient.println(
                "Conectado al seguidor solar."
            );

            showHelp();
            printPrompt();
        }
        else
        {
            newClient.println(
                "Ya existe un cliente conectado."
            );

            newClient.stop();
        }
    }

    const bool connected =
        telnetClient &&
        telnetClient.connected();

    if (
        telnetWasConnected &&
        !connected
    )
    {
        telnetWasConnected =
            false;

        if (
            activePurpose ==
                MovementPurpose::MANUAL_GOTO ||
            activePurpose ==
                MovementPurpose::MANUAL_JOG
        )
        {
            stopAllMotion(true);

            refreshTrackerState();

            Serial.println(
                "Telnet perdido: "
                "movimiento manual detenido."
            );
        }
    }

    if (!connected)
    {
        return;
    }

    while (
        telnetClient.available() >
        0
    )
    {
        const uint8_t byte =
            static_cast<uint8_t>(
                telnetClient.read()
            );

        if (
            telnetBytesToIgnore >
            0
        )
        {
            --telnetBytesToIgnore;

            continue;
        }

        if (byte == 255)
        {
            telnetBytesToIgnore = 2;

            continue;
        }

        const char c =
            static_cast<char>(
                byte
            );

        if (
            c == '\r' ||
            c == '\n'
        )
        {
            if (
                telnetCommandLength >
                0
            )
            {
                telnetCommandBuffer[
                    telnetCommandLength
                ] = '\0';

                telnetClient.print(
                    "\r\n"
                );

                processCommand(
                    telnetCommandBuffer
                );

                telnetCommandLength = 0;

                printPrompt();
            }

            continue;
        }

        if (
            c == '\b' ||
            c == 127
        )
        {
            if (
                telnetCommandLength >
                0
            )
            {
                --telnetCommandLength;

                telnetClient.print(
                    "\b \b"
                );
            }

            continue;
        }

        if (
            c >= 32 &&
            c <= 126 &&
            telnetCommandLength <
                COMMAND_BUFFER_SIZE - 1
        )
        {
            telnetCommandBuffer[
                telnetCommandLength++
            ] = c;

            telnetClient.write(
                static_cast<uint8_t>(
                    c
                )
            );
        }
    }
}

// ============================================================
// WIFI Y OTA
// ============================================================

void startNetworkServices()
{
    if (
        networkServicesStarted ||
        WiFi.status() !=
            WL_CONNECTED
    )
    {
        return;
    }

    ArduinoOTA.setHostname(
        OTA_HOSTNAME
    );

    ArduinoOTA.setPassword(
        OTA_PASSWORD
    );

    ArduinoOTA.onStart([]()
    {
        stopAllMotion(true);

        homingPhase =
            HomingPhase::IDLE;

        positionCalibrated =
            false;

        savePersistentConfig(true);

        trackerState =
            TrackerState::OTA_UPDATE;

        remotePrintln(
            "OTA iniciada. Motores detenidos."
        );
    });

    ArduinoOTA.onEnd([]()
    {
        remotePrintln(
            "OTA terminada."
        );
    });

    ArduinoOTA.onProgress(
        [](
            unsigned int progress,
            unsigned int total
        )
        {
            Serial.printf(
                "OTA: %u%%\r",
                progress *
                100U /
                total
            );
        }
    );

    ArduinoOTA.onError(
        [](ota_error_t error)
        {
            stopAllMotion(true);

            remotePrintf(
                "Error OTA: %u\n",
                static_cast<unsigned int>(
                    error
                )
            );

            refreshTrackerState();
        }
    );

    ArduinoOTA.begin();

    telnetServer.begin();
    telnetServer.setNoDelay(true);

    networkServicesStarted =
        true;

    Serial.print(
        "WiFi conectado. IP: "
    );

    Serial.println(
        WiFi.localIP()
    );

    Serial.println(
        "OTA y Telnet disponibles."
    );
}

void setupNetwork()
{
    WiFi.mode(WIFI_STA);

    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);

    WiFi.begin(
        WIFI_SSID,
        WIFI_PASSWORD
    );

    Serial.print(
        "Conectando al WiFi"
    );

    const uint32_t start =
        millis();

    while (
        WiFi.status() !=
            WL_CONNECTED &&
        millis() -
            start <
            15000
    )
    {
        delay(500);

        Serial.print('.');
    }

    Serial.println();

    if (
        WiFi.status() ==
        WL_CONNECTED
    )
    {
        startNetworkServices();
    }
    else
    {
        Serial.println(
            "WiFi no disponible. "
            "Operacion autonoma activa."
        );
    }
}

void serviceNetwork()
{
    static bool wifiWasConnected =
        false;

    static uint32_t lastReconnectAttemptMs =
        0;

    const bool connected =
        WiFi.status() ==
        WL_CONNECTED;

    if (
        wifiWasConnected &&
        !connected
    )
    {
        telnetClient.stop();

        if (
            activePurpose ==
                MovementPurpose::MANUAL_GOTO ||
            activePurpose ==
                MovementPurpose::MANUAL_JOG
        )
        {
            stopAllMotion(true);

            refreshTrackerState();

            Serial.println(
                "WiFi perdido: movimiento "
                "manual detenido."
            );
        }
        else
        {
            Serial.println(
                "WiFi perdido. Operacion "
                "autonoma continua."
            );
        }
    }

    wifiWasConnected =
        connected;

    if (connected)
    {
        if (!networkServicesStarted)
        {
            startNetworkServices();
        }

        ArduinoOTA.handle();
        serviceTelnet();

        return;
    }

    if (
        millis() -
        lastReconnectAttemptMs >=
        10000
    )
    {
        lastReconnectAttemptMs =
            millis();

        Serial.println(
            "Reintentando WiFi..."
        );

        WiFi.reconnect();
    }
}

// ============================================================
// SETUP HARDWARE
// ============================================================

void setupMotorsAndEncoders()
{
    for (
        uint8_t axis = 0;
        axis < AXIS_COUNT;
        ++axis
    )
    {
        pinMode(
            MOTOR_DIR_PINS[axis],
            OUTPUT
        );

        digitalWrite(
            MOTOR_DIR_PINS[axis],
            MOTOR_POSITIVE_DIR_LEVEL[axis]
        );

        if (
            !ledcAttach(
                MOTOR_PWM_PINS[axis],
                PWM_FREQUENCY_HZ,
                PWM_RESOLUTION_BITS
            )
        )
        {
            Serial.printf(
                "ERROR configurando PWM de %s.\n",
                axisName(axis)
            );

            while (true)
            {
                delay(1000);
            }
        }

        stopAxis(axis);
    }

    pinMode(
        ENCODER_1_A_PIN,
        INPUT_PULLUP
    );

    pinMode(
        ENCODER_1_B_PIN,
        INPUT_PULLUP
    );

    pinMode(
        ENCODER_2_A_PIN,
        INPUT_PULLUP
    );

    pinMode(
        ENCODER_2_B_PIN,
        INPUT_PULLUP
    );

    previousEncoderStates[0] =
        (
            digitalRead(
                ENCODER_1_A_PIN
            ) << 1
        ) |
        digitalRead(
            ENCODER_1_B_PIN
        );

    previousEncoderStates[1] =
        (
            digitalRead(
                ENCODER_2_A_PIN
            ) << 1
        ) |
        digitalRead(
            ENCODER_2_B_PIN
        );

    attachInterrupt(
        digitalPinToInterrupt(
            ENCODER_1_A_PIN
        ),
        encoder1Interrupt,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(
            ENCODER_1_B_PIN
        ),
        encoder1Interrupt,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(
            ENCODER_2_A_PIN
        ),
        encoder2Interrupt,
        CHANGE
    );

    attachInterrupt(
        digitalPinToInterrupt(
            ENCODER_2_B_PIN
        ),
        encoder2Interrupt,
        CHANGE
    );
}

// ============================================================
// SETUP Y LOOP
// ============================================================

void setup()
{
    Serial.begin(115200);

    delay(1000);

    Serial.println();

    Serial.println(
        "ESP32-S3 - Seguidor solar con homing fisico"
    );

    Serial.printf(
        "Conteos/vuelta antena: %.0f\n",
        ANTENNA_COUNTS_PER_REVOLUTION
    );

    if (
        rtcFaultMagic !=
        FAULT_RTC_MAGIC
    )
    {
        rtcFaultMagic =
            FAULT_RTC_MAGIC;

        consecutiveFaultRestarts = 0;

        rtcLastFaultCode =
            static_cast<uint8_t>(
                FaultCode::NONE
            );

        rtcLastFaultMessage[0] =
            '\0';
    }

    setupMotorsAndEncoders();

    initializeLimitInputs();

    delay(
        LIMIT_DEBOUNCE_MS +
        10
    );

    serviceLimitInputs();

    if (loadPersistentConfig())
    {
        Serial.println(
            "Configuracion NVS cargada y validada."
        );

        Serial.printf(
            "TRACK solicitado restaurado: %s\n",
            trackRequested
                ? "ON"
                : "OFF"
        );
    }
    else
    {
        Serial.println(
            "No existe configuracion NVS valida."
        );

        locationValid = false;
        trackRequested = false;
        rtcBackupEverProven = false;
    }

    // Nunca restaurar posicion desde NVS.
    setEncoderCount(
        AZIMUTH_AXIS,
        0
    );

    setEncoderCount(
        ELEVATION_AXIS,
        0
    );

    positionCalibrated =
        false;

    setupRtc();
    setupNetwork();

    // Siempre calibrar fisicamente al encender.
    startHoming();

    Serial.printf(
        "Estado inicial: %s\n",
        trackerStateName(
            trackerState
        )
    );
}

void loop()
{
    serviceNetwork();
    serviceLimitInputs();

    if (faultActive)
    {
        serviceFaultRestart();

        delay(1);

        return;
    }

    if (
        homingPhase !=
        HomingPhase::IDLE
    )
    {
        serviceHoming();

        delay(1);

        return;
    }

    updateAxis(
        AZIMUTH_AXIS
    );

    updateAxis(
        ELEVATION_AXIS
    );

    completeMovementPurpose();
    serviceAutomaticTracking();

    delay(1);
}
