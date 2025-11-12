#include <SPI.h>

// L3GD20H Registers
#define WHO_AM_I    0x0F
#define CTRL_REG1   0x20
#define CTRL_REG4   0x23

const int CS_PIN = 10;

// gyro values
int16_t x, y, z;

void writeReg(uint8_t reg, uint8_t value) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg & 0x7F); // Write (bit 7 = 0)
  SPI.transfer(value);
  digitalWrite(CS_PIN, HIGH);
}

uint8_t readReg(uint8_t reg) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80); // Read (bit 7 = 1)
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return value;
}

// **Burst read** for X,Y,Z (fast & correct)
void readGyro() {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x28 | 0xC0); // Auto-increment read starting at OUT_X_L
  x = (int16_t)(SPI.transfer(0) | (SPI.transfer(0) << 8));
  y = (int16_t)(SPI.transfer(0) | (SPI.transfer(0) << 8));
  z = (int16_t)(SPI.transfer(0) | (SPI.transfer(0) << 8));
  digitalWrite(CS_PIN, HIGH);
}

void setup() {
  Serial.begin(9600);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPI.begin();
  SPI.setDataMode(SPI_MODE3);             // L3GD20H requires MODE3
  SPI.setClockDivider(SPI_CLOCK_DIV8);    // Safe start → can speed up later
  SPI.setBitOrder(MSBFIRST);

  delay(100);

  // Confirm device identity
  if (readReg(WHO_AM_I) != 0xD7) {
    Serial.println("ERROR: L3GD20H NOT FOUND!");
    while (1);
  }

  // Enable all axes, normal mode, output rate 95 Hz
  writeReg(CTRL_REG1, 0b00001111);

  // Set full scale = ±2000 dps
  writeReg(CTRL_REG4, 0b00110000);

  Serial.println("L3GD20H Ready (SPI Mode)");
}

void loop() {
  readGyro();

  Serial.print(x); Serial.print('\t');
  Serial.print(y); Serial.print('\t');
  Serial.println(z);

  // delay(1);  // Optional for readability
}
