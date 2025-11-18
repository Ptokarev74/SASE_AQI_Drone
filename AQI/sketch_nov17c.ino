#include <Wire.h>
#include <Adafruit_ENS160.h>

Adafruit_ENS160 ens160;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ENS160 Air Quality Sensor Test");

  if (!ens160.begin()) {
    Serial.println("ERROR: ENS160 not detected!");
    while (1);
  }

  Serial.println("ENS160 detected!");

  // Optional but recommended
  ens160.setOperatingMode(ENS160_OPMODE_STANDARD);
}

void loop() {
  ens160.measure();

  Serial.print("AQI: ");
  Serial.print(ens160.getAQI());

  Serial.print("  TVOC: ");
  Serial.print(ens160.getTVOC());
  Serial.print(" ppb");

  Serial.print("  eCO2: ");
  Serial.print(ens160.geteCO2());
  Serial.println(" ppm");

  delay(1000);
}
