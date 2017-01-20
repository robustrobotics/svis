#include "WProgram.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define LED_PIN 13
#define IMU_DATA_SIZE 6
#define IMU_BUFFER_SIZE 10
#define IMU_PACKET_SIZE 17
#define STROBE_BUFFER_SIZE 10
#define STROBE_PACKET_SIZE 5
#define SEND_BUFFER_SIZE 64
#define SEND_HEADER_SIZE 6

// timer objects
IntervalTimer imu_timer;

// imu variables
MPU6050 mpu6050;
int32_t imu_stamp_buffer[IMU_BUFFER_SIZE];
int16_t imu_data_buffer[IMU_DATA_SIZE*IMU_BUFFER_SIZE];
uint8_t imu_buffer_head = 0;
uint8_t imu_buffer_tail = 0;
uint8_t imu_buffer_count = 0;

// strobe variables
int32_t strobe_stamp_buffer[STROBE_BUFFER_SIZE];
uint8_t strobe_buffer_head = 0;
uint8_t strobe_buffer_tail = 0;
uint8_t strobe_buffer_count = 0;

// hid usb
bool send_flag = false;
uint8_t send_buffer[SEND_BUFFER_SIZE];
uint8_t send_buffer_ind = SEND_HEADER_SIZE;
uint8_t send_count = 0;

// debug
elapsedMillis since_print;
bool led_state = false;
bool imu_debug_flag = false;
bool strobe_debug_flag = false;

void PrintIMUDataBuffer() {
  Serial.println("imu_data_buffer:");
  Serial.println("Sample:\tAx:\tAy:\tAz:\tGx:\tGy:\tGz:");
  for (int i = 0; i < IMU_BUFFER_SIZE; i++) {
    Serial.print(i);
    Serial.print(":\t");
    for (int j = 0; j < IMU_DATA_SIZE; j++) {
      Serial.print(imu_data_buffer[i*IMU_DATA_SIZE + j]);
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
  for (int i = 0; i < IMU_BUFFER_SIZE; i++) {
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
  imu_stamp_buffer[imu_buffer_head] = micros();
  mpu6050.getMotion6(&imu_data_buffer[imu_buffer_head*IMU_DATA_SIZE],
                     &imu_data_buffer[imu_buffer_head*IMU_DATA_SIZE + 1],
                     &imu_data_buffer[imu_buffer_head*IMU_DATA_SIZE + 2],
                     &imu_data_buffer[imu_buffer_head*IMU_DATA_SIZE + 3],
                     &imu_data_buffer[imu_buffer_head*IMU_DATA_SIZE + 4],
                     &imu_data_buffer[imu_buffer_head*IMU_DATA_SIZE + 5]);

  // set counts and flags
  imu_buffer_head = (imu_buffer_head + 1)%IMU_BUFFER_SIZE;

  // check tail
  if (imu_buffer_head == imu_buffer_tail) {
    imu_buffer_tail = (imu_buffer_tail + 1)%IMU_BUFFER_SIZE;
  }

  // increment count
  imu_buffer_count++;
  if (imu_buffer_count > IMU_BUFFER_SIZE) {
    imu_buffer_count = IMU_BUFFER_SIZE;
  }

  if (imu_debug_flag) {
    PrintIMUDebug();
  }
}

void PrintStrobeStampBuffer() {
  Serial.println("strobe_stamp_buffer:");
  for (int i = 0; i < STROBE_BUFFER_SIZE; i++) {
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
  strobe_stamp_buffer[strobe_buffer_head] = micros();

  // set counts and flags
  strobe_buffer_head = (strobe_buffer_head + 1)%STROBE_BUFFER_SIZE;

  // check tail
  if (strobe_buffer_head == strobe_buffer_tail) {
    strobe_buffer_tail = (strobe_buffer_tail + 1)%STROBE_BUFFER_SIZE;
  }

  // increment count
  strobe_buffer_count++;
  if (strobe_buffer_count > STROBE_BUFFER_SIZE) {
    strobe_buffer_count = STROBE_BUFFER_SIZE;
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
  Serial.println(mpu6050.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

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

int GetIMUDataBytes(uint8_t count) {
  return IMU_PACKET_SIZE*count;
}

int GetStrobeDataBytes(uint8_t count) {
  return STROBE_PACKET_SIZE*count;
}

}

void PushIMU() {
  if (send_flag) {
    return;
  }

  // get remaining space in send_buffer
  int8_t send_space = SEND_BUFFER_SIZE - (send_buffer_ind + 1);
  int num_packets = 0;

  // interrupt safe copy
  noInterrupts();

  // get number of bytes to copy
  int imu_data_bytes = GetIMUDataBytes(imu_buffer_count);

  // calculate number of packets
  if (imu_data_bytes > send_space) {
    num_packets = static_cast<int>(static_cast<float>(send_space)
                                       / static_cast<float>(IMU_PACKET_SIZE));
    send_flag = true;
  } else {
    num_packets = imu_buffer_count;
  }

  // copy data
  for (int i = 0; i < num_packets; i++) {
    send_buffer[send_buffer_ind] = 2;
    send_buffer_ind++;

    // copy stamp
    memcpy(&send_buffer[send_buffer_ind],
                &imu_stamp_buffer[imu_buffer_tail],
                sizeof(imu_stamp_buffer[imu_buffer_tail]));
    send_buffer_ind += sizeof(imu_stamp_buffer[imu_buffer_tail]);

    // copy data
    memcpy(&send_buffer[send_buffer_ind],
                &imu_data_buffer[imu_buffer_tail],
                IMU_DATA_SIZE*sizeof(imu_data_buffer[imu_buffer_tail]));
    send_buffer_ind += IMU_DATA_SIZE*sizeof(imu_data_buffer[imu_buffer_tail]);

    imu_buffer_count--;
    // check count
    if (imu_buffer_count < 0) {
      imu_buffer_count = 0;
    }

    imu_buffer_tail++;
    // check tail
    if (imu_buffer_tail >= IMU_BUFFER_SIZE) {
      imu_buffer_tail = imu_buffer_tail%IMU_BUFFER_SIZE;
    }
  }

  interrupts();
}

void PushStrobe() {
  if (send_flag) {
    return;
  }

  // get remaining space in send_buffer
  int8_t send_space = SEND_BUFFER_SIZE - (send_buffer_ind + 1);
  int num_packets = 0;

  // interrupt safe copy
  noInterrupts();

  // get number of bytes to copy
  int strobe_data_bytes = GetStrobeDataBytes(strobe_buffer_count);

  // calculate number of packets
  if (strobe_data_bytes > send_space) {
    num_packets = static_cast<int>(static_cast<float>(send_space)
                                   / static_cast<float>(STROBE_PACKET_SIZE));
    send_flag = true;
  } else {
    num_packets = strobe_buffer_count;
  }

  // copy data
  for (int i = 0; i < num_packets; i++) {
    // data id
    send_buffer[send_buffer_ind] = 1;
    send_buffer_ind++;

    // copy stamp
    memcpy(&send_buffer[send_buffer_ind],
                &strobe_stamp_buffer[strobe_buffer_tail],
                sizeof(strobe_stamp_buffer[strobe_buffer_tail]));
    send_buffer_ind += sizeof(strobe_stamp_buffer[strobe_buffer_tail]);

    strobe_buffer_count--;
    // check count
    if (strobe_buffer_count < 0) {
      strobe_buffer_count = 0;
    }

    strobe_buffer_tail++;
    // check tail
    if (strobe_buffer_tail >= STROBE_BUFFER_SIZE) {
      strobe_buffer_tail = strobe_buffer_tail%STROBE_BUFFER_SIZE;
    }
  }

  interrupts();
}

void Send() {
  // header
  send_buffer[0] = 0xAB;
  send_buffer[1] = 0xCD;

  // count
  memcpy(&send_buffer[2], &send_count, sizeof(send_count));

  // actually send the packet
  // if (RawHID.send(send_buffer, SEND_BUFFER_SIZE)) {
  //   Serial.println(send_count);
  // } else {
  //   Serial.println("E0");
  // }

  // reset
  send_count++;
  send_buffer_ind = SEND_HEADER_SIZE;
  for (int i = 0; i < SEND_BUFFER_SIZE; i++) {
    send_buffer[i] = 0;
  }

  // blink led
  if (send_count%10 == 0) {
    led_state = !led_state;
    digitalWrite(LED_PIN, led_state);
  }

  send_flag = false;
}

extern "C" int main() {
  setup();

  while (true) {
    if (strobe_buffer_count > 0) {
      PushStrobe();
    }

    if (imu_buffer_count > 0) {
      PushIMU();
    }

    if (send_flag) {
      Send();
    }

    if (since_print > 1000) {
      since_print = 0;
      Serial.println("check");
    }

    yield();  // yield() is mandatory!
  }
}
