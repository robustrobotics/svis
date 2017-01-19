#include "WProgram.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// timer object 
IntervalTimer imu_timer;

// accel and gyro variables
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

// output mode
//#define OUTPUT_READABLE_ACCELGYRO
//#define OUTPUT_BINARY_ACCELGYRO

// led variables
#define LED_PIN 13
bool blinkState = false;

// timing
long tic = 0;
long toc = 0;

unsigned long imu_count_last = 0;
unsigned long imu_count_diff = 0;
volatile unsigned long imu_count = 0;

void ReadIMU() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  imu_count++;
}

void WriteASCII() {
  Serial.print("a/g:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);
}

void WriteBinary() {
  Serial.write((uint8_t)(ax >> 8)); Serial.write((uint8_t)(ax & 0xFF));
  Serial.write((uint8_t)(ay >> 8)); Serial.write((uint8_t)(ay & 0xFF));
  Serial.write((uint8_t)(az >> 8)); Serial.write((uint8_t)(az & 0xFF));
  Serial.write((uint8_t)(gx >> 8)); Serial.write((uint8_t)(gx & 0xFF));
  Serial.write((uint8_t)(gy >> 8)); Serial.write((uint8_t)(gy & 0xFF));
  Serial.write((uint8_t)(gz >> 8)); Serial.write((uint8_t)(gz & 0xFF));
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);

  // initialize serial communication
  Serial.begin(115200);

  delay(500);

  // initialize device
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // configure Arduino LED for
  pinMode(LED_PIN, OUTPUT);

  Serial.print("Sample Rate Divisor: ");
  Serial.println(accelgyro.getRate());

  Serial.print("DLPF Mode: ");
  Serial.println(accelgyro.getDLPFMode());

  Serial.print("DHPF Mode: ");
  Serial.println(accelgyro.getDHPFMode());

  Serial.print("Gyro Range: ");
  Serial.println(accelgyro.getFullScaleGyroRange());
  
  Serial.print("Accel Range: ");
  Serial.println(accelgyro.getFullScaleAccelRange());

  // start interrupt timers
  imu_timer.begin(ReadIMU, 1000);  // microseconds
  imu_timer.priority(1);  // [0,255] with 0 as highest
}

extern "C" int main()
{
  setup();
  
  while(true) {
    if ((millis() - tic) >= 100) {
      // reset timer
      tic = millis();

      // get count
      noInterrupts();
      imu_count_diff = imu_count - imu_count_last;
      imu_count_last = imu_count;
      interrupts();

      // output count
      Serial.println(imu_count_diff);
    }

    yield();  // yield() is mandatory!
  }
}
