// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// PI controller
#include <PID_v1.h>

#define DS18B20_PIN 9

OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

double fTempSet = 65, fTempActual;
double fPidOutput; // 0-255

// PID controller parameters
#define PID_kP 25.5
#define PID_kI 1
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



}

double get_knob_pos(double fIntensity)
{
  // PID output value to servo position
  // intensity from 0 - 255
  if(fIntensity < 0 || fIntensity > 255)
  { Serial.println("Illegal intensity value"); }

  double fServoPos = map(fIntensity, 0, 255, SERVO_MAX_DEG, SERVO_MIN_DEG);

  
  return fServoPos;
  
}

void loop() {
  tempSensor.requestTemperatures();
  fTempActual = tempSensor.getTempCByIndex(0);
  fTempActual = 63;
  oPID.Compute();

  double fServoPos = get_knob_pos(fPidOutput);
  servo.write(fServoPos);

  Serial.print("Current temperature: ");
  Serial.println(fTempActual);  
  Serial.print("Set temperature: ");
  Serial.println(fTempSet);  
  Serial.print("PID output: ");
  Serial.println(fPidOutput);
  Serial.print("Knob pos: ");
  Serial.println(fServoPos);


  display.clearDisplay();
  
  display.setCursor(0,8*0); 
  display.print("Cur temp: ");
  display.print(fTempActual);
  display.print(" degC");

  display.setCursor(0, 8*1);
  display.print("Set temp: ");
  display.print(fTempSet);
  display.print(" degC");

  display.setCursor(0, 8*2);
  display.print("PID output: ");
  display.print(fPidOutput);

  display.setCursor(0, 8*3);
  display.print("Servo pos: ");
  display.print(fServoPos);
  
  display.display();

  Serial.println("---");
  delay(1000);
}

