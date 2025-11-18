#include <SPI.h>
#include <ArduinoQueue.h>

// L3GD20H Registers
#define WHO_AM_I    0x0F
#define CTRL_REG1   0x20
#define CTRL_REG4   0x23

const int CS_PIN = 10;

struct Pos {
  int x;
  int y;
  int z;
};

ArduinoQueue<Pos> intQueue(5);

int16_t x, y, z;

// --- Calibration offsets ---
int offsetX = 0;
int offsetY = 0;
int offsetZ = 0;

void writeReg(uint8_t reg, uint8_t value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F);
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t readReg(uint8_t reg) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return value;
}

void readGyroRaw() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x28 | 0xC0); // burst read

  x = (int16_t)(SPI.transfer(0) | (SPI.transfer(0) << 8));
  y = (int16_t)(SPI.transfer(0) | (SPI.transfer(0) << 8));
  z = (int16_t)(SPI.transfer(0) | (SPI.transfer(0) << 8));

  digitalWrite(CS_PIN, HIGH);
}

// ------------------------------------------------------------
//        Weighted average of queue contents
// ------------------------------------------------------------
Pos getWeightedAvg(ArduinoQueue<Pos> q) {
  int size = q.itemCount();

  if (size == 0) {
    return {0, 0, 0};
  }

  Pos items[10]; // safe, queue size max 10
  int index = 0;

  // Extract contents without modifying original
  while (!q.isEmpty()) {
    items[index++] = q.dequeue();
  }

  long sumX = 0, sumY = 0, sumZ = 0;
  long weightSum = 0;

  for (int i = 0; i < size; i++) {
    int w = i + 1; // oldest = weight 1
    sumX += items[i].x * w;
    sumY += items[i].y * w;
    sumZ += items[i].z * w;
    weightSum += w;
  }

  Pos result;
  result.x = sumX / weightSum;
  result.y = sumY / weightSum;
  result.z = sumZ / weightSum;
  return result;
}

// ------------------------------------------------------------
//                    GYRO CALIBRATION
// ------------------------------------------------------------
void calibrateGyro(int samples = 500) {
  long sumX = 0, sumY = 0, sumZ = 0;

  Serial.println("Calibrating... KEEP SENSOR STILL");
  delay(500);

  for (int i = 0; i < samples; i++) {
    readGyroRaw();
    sumX += x;
    sumY += y;
    sumZ += z;
    delay(2);
  }

  offsetX = sumX / samples;
  offsetY = sumY / samples;
  offsetZ = sumZ / samples;

  Serial.println("Calibration complete:");
  Serial.print("Offsets: ");
  Serial.print(offsetX); Serial.print(", ");
  Serial.print(offsetY); Serial.print(", ");
  Serial.println(offsetZ);
}

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(9600);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);
  SPI.setClockDivider(SPI_CLOCK_DIV8);
  SPI.setBitOrder(MSBFIRST);

  delay(100);

  if (readReg(WHO_AM_I) != 0xD7) {
    Serial.println("ERROR: L3GD20H NOT FOUND!");
    while (1);
  }

  writeReg(CTRL_REG1, 0b00001111);
  writeReg(CTRL_REG4, 0b00110000);

  Serial.println("L3GD20H Ready (SPI Mode)");

  calibrateGyro();   // <<<--- NEW
}

// ------------------------------------------------------------
// Loop
// ------------------------------------------------------------
void loop() {
  readGyroRaw();

  // Apply calibration offsets
  Pos position = {
    x - offsetX,
    y - offsetY,
    z - offsetZ
  };

  if (intQueue.isFull()) {
    intQueue.dequeue();
  }
  intQueue.enqueue(position);

  Pos avg = getWeightedAvg(intQueue);

  Serial.print("Weighted AVG: ");
  Serial.print(avg.x); Serial.print("  ");
  Serial.print(avg.y); Serial.print("  ");
  Serial.println(avg.z);

  delay(1);
}
