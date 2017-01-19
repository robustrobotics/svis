#include "WProgram.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// timer object 
IntervalTimer gyro_timer;
IntervalTimer accel_timer;

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
unsigned long gyro_count_last = 0;
unsigned long gyro_count_diff = 0;
volatile unsigned long gyro_count = 0;
unsigned long accel_count_last = 0;
unsigned long accel_count_diff = 0;
volatile unsigned long accel_count = 0;


void ReadGyro(void) {
  // accelgyro.getRotation(&gx, &gy, &gz);
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  gyro_count++;
}

void ReadAccel(void) {
  // accelgyro.getAcceleration(&ax, &ay, &az);
  accel_count++;
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

  Serial.print("Gyro Range: ");
  Serial.println(accelgyro.getFullScaleGyroRange());
  
  Serial.print("Accel Range: ");
  Serial.println(accelgyro.getFullScaleAccelRange());
  
  // start interrupt timers
  gyro_timer.begin(ReadGyro, 1000);  // microseconds
  gyro_timer.priority(0);  // [0,255] with 0 as highest
  accel_timer.begin(ReadAccel, 1000);  // microseconds
  accel_timer.priority(1);  // [0,255] with 0 as highest
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
      gyro_count_diff = gyro_count - gyro_count_last;
      gyro_count_last = gyro_count;

      accel_count_diff = accel_count - accel_count_last;
      accel_count_last = accel_count;

      interrupts();

      // output count
      Serial.print(accel_count_diff);
      Serial.print("\t");
      Serial.println(gyro_count_diff);
    }

    yield();  // yield() is mandatory!
  }
}
