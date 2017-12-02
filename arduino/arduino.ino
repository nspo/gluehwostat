// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// PI controller
#include <PID_v1.h>

#define DS18B20_PIN 5

OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

void setup() {
  Serial.begin(115200);
  tempSensor.begin();
}

void loop() {
    Serial.print("Requesting temperatures...");
  tempSensor.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(tempSensor.getTempCByIndex(0));  
}

