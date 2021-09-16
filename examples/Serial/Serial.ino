#include <witmotion_uart.h>

// Module will be used with Serial1. Avoid to use SoftwareSerial, heavy traffic possible.
#define JY901_PORT Serial2

// Make sensor's class instance
JY901_UART JY901(JY901_PORT);

void setup() {
  Serial.begin(115200);
  // Init Serial port and class instance
  JY901_PORT.begin(115200);
  JY901.begin();
}

void loop() {
  static uint32_t prevReportTime;

  // Parse incoming reports from the JY901 module
  JY901.readReport();

  if (millis() - prevReportTime >= 1333UL) {
    prevReportTime = millis();

    // Print parsed information

    // You must a specific content report item into WitMotion configuration utility
    Serial.print("\n\n\n");

    // Content -> Time
    // You can connect GPS reciever to get proper time from the air
    wmDateTime_t wmDateTime = JY901.getDateTime();
    Serial.println("Date/Time");
    Serial.print("\tYear: 20"); Serial.println(wmDateTime.year);
    Serial.print("\tMonth: "); Serial.println(wmDateTime.month);
    Serial.print("\tDay: "); Serial.println(wmDateTime.day);
    Serial.print("\tHour: "); Serial.println(wmDateTime.hour);
    Serial.print("\tMinute: "); Serial.println(wmDateTime.minute);
    Serial.print("\tSecond: "); Serial.println(wmDateTime.second);
    Serial.print("\tMillisecond: "); Serial.println(wmDateTime.millisecond);

    // Content -> Acceleration
    wmAccelerations_t wmAccelerations = JY901.getAccelerations();
    Serial.println("Acceleration");
    Serial.print("\tX: "); Serial.println(wmAccelerations.x);
    Serial.print("\tY: "); Serial.println(wmAccelerations.y);
    Serial.print("\tZ: "); Serial.println(wmAccelerations.z);

    // Content -> Velocity
    wmAngularVelocities_t wmAngularVelocities = JY901.getAngularVelocities();
    Serial.println("Angular Velocity");
    Serial.print("\tX: "); Serial.println(wmAngularVelocities.x);
    Serial.print("\tY: "); Serial.println(wmAngularVelocities.y);
    Serial.print("\tZ: "); Serial.println(wmAngularVelocities.z);

    // Content -> Angle
    wmAngles_t wmAngles = JY901.getAngles();
    Serial.println("Angle");
    Serial.print("\tX: "); Serial.println(wmAngles.roll);   // .roll() eq .x()  for wmAngles struct
    Serial.print("\tY: "); Serial.println(wmAngles.y);      // .y() eq .pitch() for wmAngles struct
    Serial.print("\tZ: "); Serial.println(wmAngles.z);      // .y() eq .yaw()   for wmAngles struct

    // Content -> Magnetism
    wmMagnetics_t wmMagnetics = JY901.getMagnetics();
    Serial.println("Magnetic Field (uT)");
    Serial.print("\tX: "); Serial.println(wmMagnetics.x);
    Serial.print("\tY: "); Serial.println(wmMagnetics.y);
    Serial.print("\tZ: "); Serial.println(wmMagnetics.z);

    // Content -> Port
    Serial.println("Port state");
    Serial.print("\tD0: "); Serial.println(JY901.getEpioStatus(epio0));
    Serial.print("\tD1: "); Serial.println(JY901.getEpioStatus(epio1));
    Serial.print("\tD2: "); Serial.println(JY901.getEpioStatus(epio2));
    Serial.print("\tD3: "); Serial.println(JY901.getEpioStatus(epio3));

    // Content -> Pressure
    Serial.println("Barometer");
    Serial.print("\tPressure (Pa): "); Serial.println(JY901.getBarometer().pressure);
    Serial.print("\tAltitude (m): "); Serial.println(JY901.getBarometer().altitude);

    // Content -> Location
    // You need GPS reciever connect to get this data
    wmGpsPosition_t wmGpsPosition = JY901.getGpsPosition();
    Serial.println("Position (GPS)");
    Serial.print("\tLongitude: "); Serial.print(wmGpsPosition.longitude.degree); Serial.print(' '); Serial.print(wmGpsPosition.longitude.minute, 4); Serial.print("' / "); Serial.println((wmGpsPosition.longitude.direction > 0x00) ? "east" : "west");
    Serial.print("\tLatitude: ");  Serial.print(wmGpsPosition.latitude.degree); Serial.print(' '); Serial.print(wmGpsPosition.latitude.minute, 4); Serial.print("' / "); Serial.println((wmGpsPosition.latitude.direction > 0x00) ? "north" : "south");

    // Content -> PDOP (sic! GPS speed pack marked as PDOP)
    // You need GPS reciever connect to get this data
    wmGpsSpeed_t wmGpsSpeed = JY901.getGpsSpeed();
    Serial.println("Ground speed (GPS)");
    Serial.print("\tAltitude (m): "); Serial.println(wmGpsSpeed.altitude);
    Serial.print("\tYaw: "); Serial.println(wmGpsSpeed.yaw);
    Serial.print("\tSpeed (km/h): "); Serial.println(wmGpsSpeed.speed, 2);

    // Content -> Quaternion
    wmQuaternions_t wmQuaternions = JY901.getQuaternions();
    Serial.println("Quaternion");
    Serial.print("\tQ0: "); Serial.println(wmQuaternions.q0, 4);
    Serial.print("\tQ1: "); Serial.println(wmQuaternions.q1, 4);
    Serial.print("\tQ2: "); Serial.println(wmQuaternions.q2, 4);
    Serial.print("\tQ3: "); Serial.println(wmQuaternions.q3, 4);

    // Content -> Accuracy (sic! PDOP pack marked as Positioning Accuracy)
    // You need GPS reciever connect to get this data
    wmGpsAccuracy_t wmGpsAccuracy = JY901.getGpsAccuracy();
    Serial.println("Accuracy (GPS)");
    Serial.print("\tSatellites quantity: "); Serial.println(wmGpsAccuracy.satellitesQuantity);
    Serial.print("\tPDOP: "); Serial.println(wmGpsAccuracy.pdop, 2);
    Serial.print("\tHDOP: "); Serial.println(wmGpsAccuracy.hdop, 2);
    Serial.print("\tVDOP: "); Serial.println(wmGpsAccuracy.vdop, 2);

      // Content -> Acceleration | Velocity | Magnetism
    Serial.print("Temperature (C): "); Serial.println(JY901.getTemperature());
  }
}
