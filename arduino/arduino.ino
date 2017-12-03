// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// PI controller
#include <PID_v1.h>

#define DS18B20_PIN 9

OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

double fTempSet = 70, fTempActual;
double fPidOutput; // 0-255

// PID controller parameters
#define PID_kP 10
#define PID_kI 0.5
#define PID_kD 0

PID oPID(&fTempActual, &fPidOutput, &fTempSet, PID_kP, PID_kI, PID_kD, DIRECT);

// display
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// Servo
#include <Servo.h>
#define SERVO_MIN_DEG 0
#define SERVO_MAX_DEG 175
#define SERVO_PIN 5
Servo servo;


void wait_for_attachment()
{
  servo.write(SERVO_MAX_DEG);
  
  for(int i=15; i>0; --i)
  {
    display.clearDisplay();
    display.setCursor(0,8*0); 
    display.print(F("ATTACH NOW"));
    display.setCursor(0,8*1); 
    display.print(F("Position: "));
    display.print(SERVO_MAX_DEG);
    display.setCursor(0,8*2); 
    display.print(F("Time left: "));
    display.print(i);
    display.display();
    delay(1000);
  }

  servo.write(SERVO_MIN_DEG);
  delay(2000);
}
void setup() {
  
  Serial.begin(115200);
  tempSensor.begin();

  tempSensor.requestTemperatures();
  fTempActual = tempSensor.getTempCByIndex(0);
  oPID.SetMode(AUTOMATIC);

  servo.attach(SERVO_PIN);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  display.display();
  
  display.clearDisplay();     
  display.setTextColor(WHITE);
  display.setTextSize(1);

  wait_for_attachment();

}

double get_knob_pos(double fIntensity)
{
  // PID output value to servo position
  // intensity from 0 - 255
  if(fIntensity < 0 || fIntensity > 255)
  { Serial.println(F("Illegal intensity value")); }

  double fServoPos = map(fIntensity, 0, 255, SERVO_MIN_DEG, SERVO_MAX_DEG);

  
  return fServoPos;
  
}

void loop() {
  tempSensor.requestTemperatures();
  fTempActual = tempSensor.getTempCByIndex(0);
  //fTempActual = 63; //fake
  oPID.Compute();

  double fServoPos = get_knob_pos(fPidOutput);
  servo.write(fServoPos);

  Serial.print(F("Current temperature: "));
  Serial.println(fTempActual);  
  Serial.print(F("Set temperature: "));
  Serial.println(fTempSet);  
  Serial.print(F("PID output: "));
  Serial.println(fPidOutput);
  Serial.print(F("Knob pos: "));
  Serial.println(fServoPos);


  display.clearDisplay();
  
  display.setCursor(0,8*0); 
  display.print(F("Cur temp: "));
  display.print(fTempActual);
  display.print(F(" degC"));

  display.setCursor(0, 8*1);
  display.print(F("Set temp: "));
  display.print(fTempSet);
  display.print(F(" degC"));

  display.setCursor(0, 8*2);
  display.print(F("PID output: "));
  display.print(fPidOutput);

  display.setCursor(0, 8*3);
  display.print(F("Servo pos: "));
  display.print(fServoPos);
  
  display.display();

  Serial.println(F("---"));
  delay(1000);
}

