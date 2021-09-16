#pragma once
#include <Arduino.h>

//#define JY901_DEBUG_ON

// !!! Manual: "The length of each address are 16bits, two bytes."
#define JY901_ADDRESS_COMMAND_SAVE                              0x00
#define JY901_ADDRESS_COMMAND_SET_CALIBRATION_MODE              0x01
#define JY901_ADDRESS_COMMAND_SET_REPORT_CONTENT                0x02
#define JY901_ADDRESS_COMMAND_SET_REPORT_RATE                   0x03
#define JY901_ADDRESS_COMMAND_SET_REPORT_BAUDRATE               0x04

#define JY901_ADDRESS_COMMAND_SET_MODE_D0                       0x0E
#define JY901_ADDRESS_COMMAND_SET_MODE_D1                       0x0F
#define JY901_ADDRESS_COMMAND_SET_MODE_D2                       0x10
#define JY901_ADDRESS_COMMAND_SET_MODE_D3                       0x11

#define JY901_ADDRESS_COMMAND_SET_PWM_HIGH_TIME_D0              0x12
#define JY901_ADDRESS_COMMAND_SET_PWM_HIGH_TIME_D1              0x13
#define JY901_ADDRESS_COMMAND_SET_PWM_HIGH_TIME_D2              0x14
#define JY901_ADDRESS_COMMAND_SET_PWM_HIGH_TIME_D3              0x15

#define JY901_ADDRESS_COMMAND_SET_PWM_PERIOD_D0                 0x16
#define JY901_ADDRESS_COMMAND_SET_PWM_PERIOD_D1                 0x17
#define JY901_ADDRESS_COMMAND_SET_PWM_PERIOD_D2                 0x18
#define JY901_ADDRESS_COMMAND_SET_PWM_PERIOD_D3                 0x19

#define JY901_ADDRESS_COMMAND_SET_I2C_ADDRESS                   0x1A
#define JY901_ADDRESS_COMMAND_SET_MODE_LED                      0x1B
#define JY901_ADDRESS_COMMAND_SET_GPS_BAUDRATE                  0x1C

#define JY901_ADDRESS_MEASUREMENT_BANDWITH                      0x1F

#define JY901_ADDRESS_COMMAND_SET_SLEEP_WAKEUP_MODE             0x22

#define JY901_ADDRESS_COMMAND_SET_MOUNT_DIRECTION               0x23
#define JY901_ADDRESS_COMMAND_SET_TRANSITION_MODE               0x24


#define JY901_ADDRESS_REGISTER_YEAR_MONTH                       0x30
#define JY901_ADDRESS_REGISTER_DAY_HOUR                         0x31
#define JY901_ADDRESS_REGISTER_MIN_SEC                          0x32
#define JY901_ADDRESS_REGISTER_MILLISECOND                      0x33
#define JY901_ADDRESS_REGISTER_ACCELERATION_X                   0x34
#define JY901_ADDRESS_REGISTER_ACCELERATION_Y                   0x35
#define JY901_ADDRESS_REGISTER_ACCELERATION_Z                   0x36
#define JY901_ADDRESS_REGISTER_ANGULAR_VELOCITY_X               0x37
#define JY901_ADDRESS_REGISTER_ANGULAR_VELOCITY_Y               0x38
#define JY901_ADDRESS_REGISTER_ANGULAR_VELOCITY_Z               0x39
#define JY901_ADDRESS_REGISTER_MAGNETIC_X                       0x3A
#define JY901_ADDRESS_REGISTER_MAGNETIC_Y                       0x3B
#define JY901_ADDRESS_REGISTER_MAGNETIC_Z                       0x3C
#define JY901_ADDRESS_REGISTER_ANGLE_X                          0x3D
#define JY901_ADDRESS_REGISTER_ANGLE_Y                          0x3E
#define JY901_ADDRESS_REGISTER_ANGLE_Z                          0x3F
#define JY901_ADDRESS_REGISTER_TEMPERATURE                      0x40
#define JY901_ADDRESS_REGISTER_STATUS_D0                        0x41
#define JY901_ADDRESS_REGISTER_STATUS_D1                        0x42
#define JY901_ADDRESS_REGISTER_STATUS_D2                        0x43
#define JY901_ADDRESS_REGISTER_STATUS_D3                        0x44
#define JY901_ADDRESS_REGISTER_BAROMETER_PRESSURE               0x45
#define JY901_ADDRESS_REGISTER_BAROMETER_ALTITUDE               0x47
#define JY901_ADDRESS_REGISTER_GPS_LONGTITUDE                   0x49
#define JY901_ADDRESS_REGISTER_GPS_LATIITUDE                    0x4B
#define JY901_ADDRESS_REGISTER_GPS_ALTITUDE                     0x4D
#define JY901_ADDRESS_REGISTER_GPS_YAW                          0x4D
#define JY901_ADDRESS_REGISTER_GPS_SPEED                        0x4F
#define JY901_ADDRESS_REGISTER_QUATERNION_0                     0x51
#define JY901_ADDRESS_REGISTER_QUATERNION_1                     0x52
#define JY901_ADDRESS_REGISTER_QUATERNION_2                     0x53
#define JY901_ADDRESS_REGISTER_QUATERNION_3                     0x54

#define JY901_ADDRESS_REGISTER_GPS_ACCURACY                     0x55

#define JY901_ADDRESS_COMMAND_SETTINGS_LOCK_UNLOCK_MODE         0x69

#define JY901_ADDRESS_REGISTER_UTC_OFFSET                       0x6B


#define JY901_COMMAND_SAVE_CONFIG                               0x0000
#define JY901_COMMAND_DROP_CONFIG                               0x0001

#define JY901_COMMAND_SLEEP_WAKEUP                              0x0001

#define JY901_COMMAND_UNLOCK                                    0xB588
#define JY901_COMMAND_LOCK                                      0x77A5

#define JY901_UTC_BASE                                          0x0C

typedef enum : uint8_t { 
      epio0              = 0x00,
      epio1              = 0x01,
      epio2              = 0x02,
      epio3              = 0x03
} wmExtendedPort_t;

typedef enum : uint16_t { 
      aInput             = 0x0000, 
      dInput             = 0x0001, 
      dHigh              = 0x0002, 
      dLow               = 0x0003, 
      dPwm               = 0x0004, 
      gpsInput           = 0x0005
} wmExtendedPortMode_t;      
                             
typedef enum : uint16_t { 
      cEnd               = 0x0000, 
      cGyroAccel         = 0x0001, 
      cMagnetic          = 0x0002, 
      cAltitudeZero      = 0x0003, 
      cAxisZAngleZero    = 0x0004,
      cAxisXYAnglesZero  = 0x0008,
} wmCalibrationMode_t;

typedef enum : uint16_t { 
      RR_0_2_Hz          = 0x0001, 
      RR_0_5_Hz          = 0x0002, 
      RR_1_Hz            = 0x0003, 
      RR_2_Hz            = 0x0004, 
      RR_5_Hz            = 0x0005, 
      RR_10_Hz           = 0x0006, 
      RR_20_Hz           = 0x0007, 
      RR_50_Hz           = 0x0008, 
      RR_100_Hz          = 0x0009, 
      RR_125_Hz          = 0x000A, 
      RR_200_Hz          = 0x000B, 
      RR_Single          = 0x000C, // Report will be send once after this command using or after wakeup, or after 'Send me report command'
      RR_Disabled        = 0x000D
} wmReportRate_t;              
                               
typedef enum : uint16_t { 
      RBR_4800           = 0x0001, 
      RBR_9600           = 0x0002, 
      RBR_19200          = 0x0003, 
      RBR_38400          = 0x0004, 
      RBR_57600          = 0x0005, 
      RBR_115200         = 0x0006, 
      RBR_230400         = 0x0007, 
      RBR_460800         = 0x0008, 
      RBR_921600         = 0x0009
} wmReportBaudrate_t;         
                              
typedef enum : uint16_t {      
      GBR_2400           = 0x0000, 
      GBR_4800           = 0x0001, 
      GBR_9600           = 0x0002, 
      GBR_19200          = 0x0003, 
      GBR_38400          = 0x0004, 
      GBR_57600          = 0x0005, 
      GBR_115200         = 0x0006, 
      GBR_230400         = 0x0007, 
      GBR_460800         = 0x0008, 
      GBR_921600         = 0x0009
} wmGpsBaudrate_t;           

typedef enum : uint16_t {      
      MBW_256_Hz         = 0x0000, 
      MBW_188_Hz         = 0x0001, 
      MBW_98_Hz          = 0x0002, 
      MBW_42_Hz          = 0x0003, 
      MBW_20_Hz          = 0x0004, 
      MBW_10_Hz          = 0x0005, 
      MBW_5_Hz           = 0x0006, 
} wmMeasurementBandwidth_t;           
                             
                             
typedef enum : uint16_t { 
      lOn                = 0x0000, 
      lOff               = 0x0001 
} wmLedMode_t;

typedef enum : uint16_t { 
      dHorizontal        = 0x0000, 
      dVertical          = 0x0001 
} wmMountDirection_t;

typedef enum : uint16_t { 
      axis9              = 0x0000, 
      axis6              = 0x0001 
} wmTransitionMode_t;

typedef enum : uint16_t { 
      cDateTime          = 0x0001, 
      cAccelerations     = 0x0002,
      cAngularVelocities = 0x0004,
      cAngles            = 0x0008,
      cMagnetics         = 0x0010,
      cEpioStatus        = 0x0020,
      cBarometer         = 0x0040,
      cGpsPosition       = 0x0080,
      cGpsSpeed          = 0x0100,
      cQuaternions       = 0x0200,
      cGpsAccuracy       = 0x0400
} wmReportContent_t;

typedef enum : int8_t { 
      west               = -0x01, 
      east               = 0x01 
} wmGpsLongitudeDirection_t;

typedef enum : int8_t { 
      south              = -0x01, 
      north              = 0x01
} wmGpsLatitudeDirection_t;

typedef struct {
  uint8_t  year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  second;
  uint16_t millisecond;
} wmRawDateTime_t;

// Pressure value in Pa
// Altitude value in cm (centimeters)
typedef struct {
  int32_t  pressure;
  int32_t  altitude;
} wmRawBarometer_t;

typedef struct {
  int32_t  longitude;
  int32_t  latitude;
} wmRawGpsPosition_t;

typedef struct {
  int16_t  altitude;
  int16_t  yaw;
  int32_t  speed;
} wmRawGpsSpeed_t;

typedef struct {
  uint16_t satellitesQuantity;
  uint16_t pdop;
  uint16_t hdop;
  uint16_t vdop;
} wmRawGpsAccuracy_t;

typedef struct { int16_t  x, y, z; } wmRawAccelerations_t;

typedef struct { int16_t  x, y, z; } wmRawAngularVelocities_t;

typedef struct { int16_t  x, y, z; } wmRawAngles_t;

typedef struct { int16_t  x, y, z; } wmRawMagnetics_t;

typedef struct { uint16_t D0, D1, D2, D3; } wmRawExtendedPorts_t;

typedef struct { int16_t  q0, q1, q2, q3; } wmRawQuaternions_t;

typedef struct {
  wmRawDateTime_t          dateTime;
  wmRawAccelerations_t     accelerations;
  wmRawAngularVelocities_t angularVelocities;
  wmRawMagnetics_t         magnetics;
  wmRawAngles_t            angles;
  int16_t                  temperature;
  int16_t                  version;
  union {
    wmRawExtendedPorts_t   extendedPorts;
    uint16_t               extendedPort[0x04];
  };
  wmRawBarometer_t         barometer;
  wmRawGpsPosition_t       gpsPosition;
  wmRawGpsSpeed_t          gpsSpeed;
  wmRawQuaternions_t       quaternions;
  wmRawGpsAccuracy_t       gpsAccuracy;
} metrics_t;

struct wmAccelerations_s { float x, y, z; };

struct wmAngularVelocities_s { float x, y, z; };

struct wmAngles_s {
  union {
    float x;
    float roll;
  };
  union {
    float y;
    float pitch;
  };
  union {
    float z;
    float yaw;
  };
};

struct wmBarometer_s {
  int32_t pressure;
  float   altitude;
};

struct wmGpsPosition_s {  
  struct {
    uint8_t                   degree;
    float                     minute; 
    wmGpsLongitudeDirection_t direction;
  } longitude;
  struct {
    uint8_t                  degree;
    float                    minute; 
    wmGpsLatitudeDirection_t direction;
  } latitude;
};

struct wmGpsSpeed_s {
  float  altitude;
  float  yaw;
  float speed;
};

typedef struct {
  float q0;
  float q1;
  float q2;
  float q3;
} wmQuaternions_t;

typedef struct {
  uint16_t satellitesQuantity;
  float pdop;
  float hdop;
  float vdop;
} wmGpsAccuracy_t;

typedef struct wmAccelerations_s wmAccelerations_t;
typedef struct wmAngularVelocities_s wmAngularVelocities_t;
typedef struct wmAngles_s wmAngles_t;
typedef struct wmBarometer_s wmBarometer_t;
typedef struct wmGpsPosition_s wmGpsPosition_t;
typedef struct wmGpsSpeed_s wmGpsSpeed_t;
typedef wmRawDateTime_t wmDateTime_t;
typedef wmRawMagnetics_t wmMagnetics_t;
typedef wmRawExtendedPorts_t wmExtendedPorts_t;

class JY901_BASE {
  public:
    JY901_BASE() { ; }

    virtual uint8_t begin();

    // Raw data getters
    wmRawDateTime_t          getRawDateTime();
    wmRawAccelerations_t     getRawAccelerations();
    wmRawAngularVelocities_t getRawAngularVelocities();
    wmRawAngles_t            getRawAngles();
    wmRawMagnetics_t         getRawMagnetics();
    wmRawExtendedPorts_t     getRawEpioses();
    int16_t                  getRawTemperature();
    wmRawBarometer_t         getRawBarometer();
    wmRawGpsPosition_t       getRawGpsPosition();
    wmRawGpsSpeed_t          getRawGpsSpeed();
    wmRawQuaternions_t       getRawQuaternions();
    wmRawGpsAccuracy_t       getRawGpsAccuracy();

    // Normalized data getters
    wmDateTime_t             getDateTime();
    wmAccelerations_t        getAccelerations();
    wmAngularVelocities_t    getAngularVelocities();
    wmAngles_t               getAngles();
    wmMagnetics_t            getMagnetics();
    wmExtendedPorts_t        getEpioses();
    uint16_t                 getEpioStatus(wmExtendedPort_t);
    uint16_t                 getEpioStatus(uint8_t);
    uint16_t                 getVersion();
    float                    getTemperature();
    wmBarometer_t            getBarometer();
    wmGpsPosition_t          getGpsPosition();
    wmGpsSpeed_t             getGpsSpeed();
    wmQuaternions_t          getQuaternions();
    wmGpsAccuracy_t          getGpsAccuracy();

  private:
    virtual uint8_t sendData(uint8_t, uint16_t);

  protected:

    metrics_t metrics;

};
