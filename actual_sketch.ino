#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "ICM_20948.h"

// ================= CONFIG =================
#define UNIT_ID 3
#define SAMPLE_HZ 50
#define DT_US (1000000 / SAMPLE_HZ)
// ==========================================

// ---------- SparkFun IMU ----------
ICM_20948_I2C imu;

// ---------- ESP-NOW PACKET ----------
typedef struct __attribute__((packed)) {
  uint8_t unit_id;
  uint32_t t_ms;
  float ax, ay, az;
  float gx, gy, gz;
} IMUPacket;

uint8_t broadcastAddr[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

// ---------- RECEIVE FROM UNITS 1 & 2 ----------
void onReceive(const esp_now_recv_info *info,
               const uint8_t *data,
               int len) {
  if (len != sizeof(IMUPacket)) return;

  IMUPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  // Forward wireless data to Python
  Serial.printf(
    "%d,%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
    pkt.unit_id,
    pkt.t_ms,
    pkt.ax, pkt.ay, pkt.az,
    pkt.gx, pkt.gy, pkt.gz
  );
}

// ---------- TIMING ----------
unsigned long nextMicros = 0;

void setup() {
  Serial.begin(115200);

  // ---------- I2C ----------
  Wire.begin(21, 22);
  Wire.setClock(400000);

  // ---------- SparkFun IMU ----------
  bool ok = false;
  while (!ok) {
    imu.begin(Wire, 1); // AD0 = 1
    if (imu.status == ICM_20948_Stat_Ok) {
      ok = true;
    } else {
      delay(500);
    }
  }

  // ---------- ESP-NOW ----------
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    while (1);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddr, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  esp_now_register_recv_cb(onReceive);

  Serial.println("BASE_READY");
  nextMicros = micros();
}

void loop() {
  unsigned long now = micros();
  if ((long)(now - nextMicros) < 0) return;
  nextMicros += DT_US;

  if (!imu.dataReady()) return;

  imu.getAGMT();

  // Accel: mg → g (normalized-ish like other units)
  float ax = imu.accX() / 1000.0f;
  float ay = imu.accY() / 1000.0f;
  float az = imu.accZ() / 1000.0f;

  float an = sqrt(ax*ax + ay*ay + az*az);
  if (an > 0) { ax/=an; ay/=an; az/=an; }

  // Gyro: DPS → rad/s
  float gx = imu.gyrX() * DEG_TO_RAD;
  float gy = imu.gyrY() * DEG_TO_RAD;
  float gz = imu.gyrZ() * DEG_TO_RAD;

  // Send Unit 3 data to Python
  Serial.printf(
    "%d,%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f\n",
    UNIT_ID,
    millis(),
    ax, ay, az,
    gx, gy, gz
  );
}
