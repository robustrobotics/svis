#include "WProgram.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define LED_PIN 13

// timer objects
IntervalTimer imu_timer;

// accel and gyro variables
MPU6050 mpu6050;
int16_t imu_data[6];  // [ax, ay, ax, gx, gy, gz]

volatile bool imu_flag = false;
volatile bool strobe_flag = false;

void ReadIMU() {
  mpu6050.getMotion6(&imu_data[0], &imu_data[1], &imu_data[2], &imu_data[3], &imu_data[4], &imu_data[5]);
  imu_flag = true;
}

void OnStrobe() {
  strobe_flag = true;
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  // initialize serial communication
  Serial.begin(115200);

  delay(500);

  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu6050.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);

  Serial.print("Sample Rate Divisor: ");
  Serial.println(mpu6050.getRate());

  Serial.print("DLPF Mode: ");
  Serial.println(mpu6050.getDLPFMode());

  Serial.print("DHPF Mode: ");
  Serial.println(mpu6050.getDHPFMode());

  Serial.print("Gyro Range: ");
  Serial.println(mpu6050.getFullScaleGyroRange());
  
  Serial.print("Accel Range: ");
  Serial.println(mpu6050.getFullScaleAccelRange());

  // setup interrupt timers
  imu_timer.begin(ReadIMU, 1000);  // microseconds
  imu_timer.priority(0);  // [0,255] with 0 as highest

  // setup pin interrupt
  // TODO(jakeware): What is the priority of this?
  attachInterrupt(7, OnStrobe, RISING);  // attach pin 7 to interrupt
}

void SendIMU() {
  // make threadsafe copy
  int16_t imu_data_temp[6];
  noInterrupts();
  for(int i = 0; i < 6; i++) {
    imu_data_temp[i] = imu_data[i];
  }
  interrupts();

  // send data

  imu_flag = false;
}

void SendStrobe() {
  // send data
  
  strobe_flag = false;
}

extern "C" int main()
{
  setup();
  
  while(true) {
    if (imu_flag) {
      SendIMU();
    }

    if (strobe_flag) {
      SendStrobe();
    }

    yield();  // yield() is mandatory!
  }
}
