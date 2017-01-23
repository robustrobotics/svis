#include "WProgram.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// hardware
#define LED_PIN 13  // pin for on-board led

/* packet structure
[0-1]: send_count
[2]: imu_count
[3]: strobe_count
[4-19]: imu packet 1
[20-35]: imu packet 2
[36-51]: imu packet 3
[52-56]: strobe packet 1
[57-61]: strobe packet 2
[62-63]: checksum
*/

// hid usb packet sizes
const int imu_data_size = 6;  // (int16_t) [ax, ay, az, gx, gy, gz]
const int imu_buffer_size = 10;  // store 10 samples (imu_stamp, imu_data) in circular buffers
const int imu_packet_size = 16;  // (int8_t) [imu_stamp[0], ... , imu_stamp[3], imu_data[0], ... , imu_data[11]]
const int strobe_buffer_size = 10;  // store 10 samples (strobe_stamp, strobe_count) in circular buffers
const int strobe_packet_size = 5;  // (int8_t) [strobe_stamp[0], ... , strobe_stamp[3], strobe_count]
const int send_buffer_size = 64;  // (int8_t) size of HID USB packets
const int send_header_size = 4;  // (int8_t) [send_count[0], send_count[1], imu_count, strobe_count];

// hid usb packet indices
const int send_count_index = 0;
const int imu_count_index = 2;
const int strobe_count_index = 3;
const int imu_index[3] = {4, 20, 36};
const int strobe_index[2] = {52, 57};
const int checksum_index = 62;

// hid usb
uint8_t send_buffer[send_buffer_size];
uint16_t send_count = 0;
uint8_t send_errors = 0;
uint8_t strobe_packet_count = 0;
uint8_t imu_packet_count = 0;

// imu variables
IntervalTimer imu_timer;
MPU6050 mpu6050;
int32_t imu_stamp_buffer[imu_buffer_size];
int16_t imu_data_buffer[imu_data_size*imu_buffer_size];
uint8_t imu_buffer_head = 0;
uint8_t imu_buffer_tail = 0;
uint8_t imu_buffer_count = 0;

// strobe variables
int32_t strobe_stamp_buffer[strobe_buffer_size];
uint8_t strobe_count = 0;
uint8_t strobe_count_buffer[strobe_buffer_size];
uint8_t strobe_buffer_head = 0;
uint8_t strobe_buffer_tail = 0;
uint8_t strobe_buffer_count = 0;

// debug
elapsedMillis since_print;
bool led_state = false;
bool imu_debug_flag = false;
bool strobe_debug_flag = false;
bool send_debug_flag = false;

void PrintIMUDataBuffer() {
  Serial.println("imu_data_buffer:");
  Serial.println("Sample:\tAx:\tAy:\tAz:\tGx:\tGy:\tGz:");
  for (int i = 0; i < imu_buffer_size; i++) {
    Serial.print(i);
    Serial.print(":\t");
    for (int j = 0; j < imu_data_size; j++) {
      Serial.print(imu_data_buffer[i*imu_data_size + j]);
      Serial.print("\t");
    }

    // mark head location
    if (i == imu_buffer_head) {
      Serial.print("\tH");
    }

    // mark tail location
    if (i == imu_buffer_tail) {
      Serial.print("\tT");
    }

    Serial.println();
  }
}

void PrintIMUStampBuffer() {
  Serial.println("imu_stamp_buffer:");
  for (int i = 0; i < imu_buffer_size; i++) {
    Serial.print(i);
    Serial.print(":\t");
    Serial.print(imu_stamp_buffer[i]);

    // mark head location
    if (i == imu_buffer_head) {
      Serial.print("\tH");
    }

    // mark tail location
    if (i == imu_buffer_tail) {
      Serial.print("\tT");
    }

    Serial.println();
  }
}

void PrintIMUDebug() {
  PrintIMUStampBuffer();
  PrintIMUDataBuffer();
  Serial.print("imu_buffer_head: ");
  Serial.println(imu_buffer_head);
  Serial.print("imu_buffer_tail: ");
  Serial.println(imu_buffer_tail);
  Serial.print("imu_buffer_count: ");
  Serial.println(imu_buffer_count);
}

void ReadIMU() {
  // imu timestamp
  imu_stamp_buffer[imu_buffer_head] = micros();

  // imu data
  mpu6050.getMotion6(&imu_data_buffer[imu_buffer_head*imu_data_size],
                     &imu_data_buffer[imu_buffer_head*imu_data_size + 1],
                     &imu_data_buffer[imu_buffer_head*imu_data_size + 2],
                     &imu_data_buffer[imu_buffer_head*imu_data_size + 3],
                     &imu_data_buffer[imu_buffer_head*imu_data_size + 4],
                     &imu_data_buffer[imu_buffer_head*imu_data_size + 5]);

  // set counts and flags
  imu_buffer_head = (imu_buffer_head + 1)%imu_buffer_size;

  // check tail
  if (imu_buffer_head == imu_buffer_tail) {
    imu_buffer_tail = (imu_buffer_tail + 1)%imu_buffer_size;
  }

  // increment count
  imu_buffer_count++;
  if (imu_buffer_count > imu_buffer_size) {
    imu_buffer_count = imu_buffer_size;
  }

  if (imu_debug_flag) {
    PrintIMUDebug();
  }
}

void PrintStrobeStampBuffer() {
  Serial.println("strobe_stamp_buffer:");
  for (int i = 0; i < strobe_buffer_size; i++) {
    Serial.print(i);
    Serial.print(":\t");
    Serial.print(strobe_stamp_buffer[i]);

    // mark head location
    if (i == strobe_buffer_head) {
      Serial.print("\tH");
    }

    // mark tail location
    if (i == strobe_buffer_tail) {
      Serial.print("\tT");
    }

    Serial.println();
  }
}

void PrintStrobeDebug() {
  PrintStrobeStampBuffer();
  Serial.print("strobe_buffer_head: ");
  Serial.println(strobe_buffer_head);
  Serial.print("strobe_buffer_tail: ");
  Serial.println(strobe_buffer_tail);
  Serial.print("strobe_buffer_count: ");
  Serial.println(strobe_buffer_count);
}

void ReadStrobe() {
  // strobe timestamp
  strobe_stamp_buffer[strobe_buffer_head] = micros();

  // strobe count
  strobe_count_buffer[strobe_buffer_head] = strobe_count;
  strobe_count = (strobe_count + 1)%128;  // roll over at 128 like flea3

  // set counts and flags
  strobe_buffer_head = (strobe_buffer_head + 1)%strobe_buffer_size;

  // check tail
  if (strobe_buffer_head == strobe_buffer_tail) {
    strobe_buffer_tail = (strobe_buffer_tail + 1)%strobe_buffer_size;
  }

  // increment count
  strobe_buffer_count++;
  if (strobe_buffer_count > strobe_buffer_size) {
    strobe_buffer_count = strobe_buffer_size;
  }

  if (strobe_debug_flag) {
    PrintStrobeDebug();
  }
}

void InitMPU6050() {
  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu6050.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu6050.testConnection() ?
                 "MPU6050 connection successful" : "MPU6050 connection failed");

  // print registers
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
}

void InitComms() {
  // initialize serial communication
  Serial.begin(115200);

  // initialize i2c communication
  Wire.begin();
  Wire.setClock(400000);

  delay(500);
}

void InitGPIO() {
  // configure onboard LED
  pinMode(LED_PIN, OUTPUT);
}

void InitInterrupts() {
  // setup interrupt timers
  imu_timer.begin(ReadIMU, 1000);  // microseconds
  imu_timer.priority(0);  // [0,255] with 0 as highest

  // setup pin interrupt
  // TODO(jakeware): What is the priority of this?
  attachInterrupt(7, ReadStrobe, RISING);  // attach pin 7 to interrupt
}

void Blink() {
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWriteFast(LED_PIN, LOW);
  delay(500);
}

void setup() {
  InitComms();
  InitGPIO();
  Blink();
  InitMPU6050();
  InitInterrupts();
}

void PrintSendBuffer() {
  Serial.println("send_buffer: ");
  for (int i = 0; i < send_buffer_size; i++) {
    Serial.print(i);
    Serial.print("\t");
    Serial.print(send_buffer[i]);
    Serial.println();
  }
}

void PushIMU() {
  // interrupt safe copy
  noInterrupts();

  // calculate number of packets
  if (imu_buffer_count >= 0 && imu_buffer_count <= 3) {
    imu_packet_count = imu_buffer_count;
  } else if (imu_buffer_count > 3) {
    imu_packet_count = 3;
  } else {
    // totally fucked
    // Serial.println("num_packets < 0");
    imu_packet_count = 0;
  }

  // copy data
  for (int i = 0; i < imu_packet_count; i++) {
    // copy stamp
    memcpy(&send_buffer[imu_index[i]],
                &imu_stamp_buffer[imu_buffer_tail],
                sizeof(imu_stamp_buffer[imu_buffer_tail]));

    // copy data
    memcpy(&send_buffer[imu_index[i] + 4],
                &imu_data_buffer[imu_buffer_tail],
                imu_data_size*sizeof(imu_data_buffer[imu_buffer_tail]));

    imu_buffer_count--;
    // check count
    if (imu_buffer_count < 0) {
      imu_buffer_count = 0;
    }

    imu_buffer_tail++;
    // check tail
    if (imu_buffer_tail >= imu_buffer_size) {
      imu_buffer_tail = imu_buffer_tail%imu_buffer_size;
    }
  }

  // enable interrupts
  interrupts();
}

void PushStrobe() {
  // interrupt safe copy
  noInterrupts();

  // calculate number of packets
  if (strobe_buffer_count >= 0 && strobe_buffer_count <= 2) {
    strobe_packet_count = strobe_buffer_count;
  } else if (strobe_buffer_count > 2) {
    strobe_packet_count = 2;
  } else {
    // totally fucked
    // Serial.println("num_packets < 0");
    strobe_packet_count = 0;
  }

  // copy data
  for (int i = 0; i < strobe_packet_count; i++) {
    // copy stamp
    memcpy(&send_buffer[strobe_index[i]],
                &strobe_stamp_buffer[strobe_buffer_tail],
                sizeof(strobe_stamp_buffer[strobe_buffer_tail]));

    // copy count
    send_buffer[strobe_index[i] + 4] = strobe_count_buffer[strobe_buffer_tail];

    strobe_buffer_count--;
    // check count
    if (strobe_buffer_count < 0) {
      strobe_buffer_count = 0;
    }

    strobe_buffer_tail++;
    // check tail
    if (strobe_buffer_tail >= strobe_buffer_size) {
      strobe_buffer_tail = strobe_buffer_tail%strobe_buffer_size;
    }
  }

  // enable interrupts
  interrupts();
}

void Send() {
  // send_count
  memcpy(&send_buffer[send_count_index], &send_count, sizeof(send_count));

  // data
  PushIMU();
  PushStrobe();
  
  // packet_counts
  send_buffer[imu_count_index] = imu_packet_count;
  send_buffer[strobe_count_index] = strobe_packet_count;

  // checksum
  uint16_t checksum = 0;
  for (int i = 0; i < send_buffer_size; i++) {
    checksum += send_buffer[i];
  }
  memcpy(&send_buffer[checksum_index], &checksum, sizeof(checksum));

  // send packet
  if (RawHID.send(send_buffer, send_buffer_size)) {
    // blink led
    if (send_count%10 == 0) {
      led_state = !led_state;
      digitalWrite(LED_PIN, led_state);
    }
  } else {
    send_errors++;
  }

  // debug print
  if (send_debug_flag) {
    PrintSendBuffer();
  }

  // reset
  send_count++;
  imu_packet_count = 0;
  strobe_packet_count = 0;
  for (int i = 0; i < send_buffer_size; i++) {
    send_buffer[i] = 0;
  }
}

extern "C" int main() {
  setup();

  while (true) {
    if (imu_buffer_count >= 3) {
      Send();
    }

    if (since_print > 1000) {
      since_print = 0;
      // Serial.println("check");
    }

    yield();  // yield() is mandatory!
  }
}
