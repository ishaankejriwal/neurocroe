#include "ICM_20948.h"
#include <Wire.h>

#define SERIAL_PORT Serial
#define WIRE_PORT Wire

// Address settings
// AD0_VAL 1 = address 0x69
// AD0_VAL 0 = address 0x68
#define BACK_AD0_VAL 1   // lower-back sensor, 0x69
#define HIP_AD0_VAL 0    // hip sensor, 0x68

ICM_20948_I2C backIMU;
ICM_20948_I2C hipIMU;

void setup() {
  SERIAL_PORT.begin(115200);
  delay(1500);

  SERIAL_PORT.println();
  SERIAL_PORT.println("Starting NeuroCore two-IMU stream...");

  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);

  bool backReady = false;
  bool hipReady = false;

  while (!backReady) {
    backIMU.begin(WIRE_PORT, BACK_AD0_VAL);

    SERIAL_PORT.print("Back IMU init: ");
    SERIAL_PORT.println(backIMU.statusString());

    if (backIMU.status == ICM_20948_Stat_Ok) {
      backReady = true;
    } else {
      SERIAL_PORT.println("Retrying back IMU...");
      delay(1000);
    }
  }

  while (!hipReady) {
    hipIMU.begin(WIRE_PORT, HIP_AD0_VAL);

    SERIAL_PORT.print("Hip IMU init: ");
    SERIAL_PORT.println(hipIMU.statusString());

    if (hipIMU.status == ICM_20948_Stat_Ok) {
      hipReady = true;
    } else {
      SERIAL_PORT.println("Retrying hip IMU...");
      delay(1000);
    }
  }

  SERIAL_PORT.println("Both IMUs initialized successfully.");
  SERIAL_PORT.println("Streaming FRAME data...");
}

void printIMU(String label, ICM_20948_I2C &imu) {
  SERIAL_PORT.print(label);

  SERIAL_PORT.print(" AccX=");
  SERIAL_PORT.print(imu.accX(), 2);
  SERIAL_PORT.print(" AccY=");
  SERIAL_PORT.print(imu.accY(), 2);
  SERIAL_PORT.print(" AccZ=");
  SERIAL_PORT.print(imu.accZ(), 2);

  SERIAL_PORT.print(" GyrX=");
  SERIAL_PORT.print(imu.gyrX(), 2);
  SERIAL_PORT.print(" GyrY=");
  SERIAL_PORT.print(imu.gyrY(), 2);
  SERIAL_PORT.print(" GyrZ=");
  SERIAL_PORT.print(imu.gyrZ(), 2);

  SERIAL_PORT.print(" MagX=");
  SERIAL_PORT.print(imu.magX(), 2);
  SERIAL_PORT.print(" MagY=");
  SERIAL_PORT.print(imu.magY(), 2);
  SERIAL_PORT.print(" MagZ=");
  SERIAL_PORT.print(imu.magZ(), 2);

  SERIAL_PORT.print(" Temp=");
  SERIAL_PORT.print(imu.temp(), 2);
}

void loop() {
  bool backReady = backIMU.dataReady();
  bool hipReady = hipIMU.dataReady();

  if (backReady) {
    backIMU.getAGMT();
  }

  if (hipReady) {
    hipIMU.getAGMT();
  }

  if (backReady && hipReady) {
    SERIAL_PORT.print("FRAME ");

    printIMU("BACK", backIMU);
    SERIAL_PORT.print(" | ");
    printIMU("HIP", hipIMU);

    SERIAL_PORT.println();

    delay(30);
  } else {
    SERIAL_PORT.println("Waiting for both sensors...");
    delay(100);
  }
}

