// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

// PI controller
#include <PID_v1.h>

#define DS18B20_PIN 3

OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

double fTempSet = 70, fTempActual;
double fPidOutput; // 0-255

// PID controller parameters
#define PID_kP_default 10
#define PID_kI_default 0.5
#define PID_kD_default 0

PID oPID(&fTempActual, &fPidOutput, &fTempSet, PID_kP_default, PID_kI_default, PID_kD_default, DIRECT);

// display
#include <MenuSystem.h>
#include <LiquidCrystal.h>

int lcd_key = 0;
int adc_key_in1 = 0, adc_key_in2 = 0;
#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5

int nLastButton = btnNONE;

int read_LCD_buttons()
{
    // get current button state
    // with simple debouncing workaround
    adc_key_in1 = 0;
    adc_key_in2 = 1023;

    while (abs(adc_key_in1 - adc_key_in2) > 10)
    {
        // read again until value stays the same within range
        adc_key_in1 = analogRead(0);
        delay(5);
        adc_key_in2 = analogRead(0);
    }

    if (adc_key_in1 > 1000)
        return btnNONE; // We make this the 1st option for speed reasons since it will be the most likely result

    if (adc_key_in1 < 50)
        return btnRIGHT;
    if (adc_key_in1 < 195)
        return btnUP;
    if (adc_key_in1 < 380)
        return btnDOWN;
    if (adc_key_in1 < 555)
        return btnLEFT;
    if (adc_key_in1 < 790)
        return btnSELECT;

    return btnNONE; // when all others fail, return this...
}

LiquidCrystal lcd = LiquidCrystal(8, 9, 4, 5, 6, 7);

class DefaultRenderer : public MenuComponentRenderer
{
  public:
    void render(Menu const &menu) const
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(menu.get_name());
        lcd.setCursor(0, 1);
        menu.get_current_component()->render(*this);
    }

    void render_menu_item(MenuItem const &menu_item) const
    {
        lcd.print(menu_item.get_name());
    }

    void render_back_menu_item(BackMenuItem const &menu_item) const
    {
        lcd.print(menu_item.get_name());
    }

    void render_numeric_menu_item(NumericMenuItem const &menu_item) const
    {
        lcd.print(menu_item.get_name());
        if (menu_item.has_focus())
        {
            lcd.print(":>");
        }
        else
        {
            lcd.print(": ");
        }
        lcd.print(menu_item.get_formatted_value());
    }

    void render_numeric_display_menu_item(NumericDisplayMenuItem const &menu_item) const
    {
        lcd.print(menu_item.get_name());
        lcd.print(": ");
        lcd.print(menu_item.get_formatted_value());
    }

    void render_menu(Menu const &menu) const
    {
        lcd.print(menu.get_name());
    }
};

DefaultRenderer my_renderer;

// Menu variables
MenuSystem ms(my_renderer);

Menu menu_i1("Temperature [dC]");
NumericDisplayMenuItem menu_tempActual("Actual", NULL, NULL, 23.2);
NumericMenuItem menu_tempSet("Set", NULL, 69, 0, 99, 0.5, NULL);

Menu menu_i2("PID values");
NumericDisplayMenuItem menu_pidOut("PID out", NULL, NULL, 999);
NumericDisplayMenuItem menu_servoPos("Servo pos", NULL, NULL, 999);
NumericMenuItem menu_pidKP("k_P", NULL, 10, 0, 99, 0.5, NULL);
NumericMenuItem menu_pidKI("k_I", NULL, 0.5, 0, 99, 0.02, NULL);
NumericMenuItem menu_pidKD("k_D", NULL, 0, 0, 99, 0.1, NULL);
MenuItem menu_pidReset("Reset PID state", NULL);

Menu menu_i3("Manual mode[deg]");
NumericMenuItem menu_manualMode("Manual", NULL, 0, 0, 1, 1.0, &MenuHelpers::format_int);
NumericMenuItem menu_manualServoPos("Servo", NULL, 0, 0, 175, 2.5, NULL);

Menu menu_i4("Misc debug");
NumericDisplayMenuItem menu_loopDelay("LoopLag", NULL, &MenuHelpers::format_int, -1);

// Servo
#include <Servo.h>

#define SERVO_MIN_DEG 0
#define SERVO_MAX_DEG 175
#define SERVO_PIN 2
Servo servo;

double fServoPos;
unsigned long nLastLoopExecTime = 0;

void setup_menu()
{
    lcd.clear();

    ms.get_root_menu().add_menu(&menu_i1);
    menu_i1.add_item(&menu_tempActual);
    menu_i1.add_item(&menu_tempSet);

    ms.get_root_menu().add_menu(&menu_i2);
    menu_i2.add_item(&menu_pidOut);
    menu_i2.add_item(&menu_servoPos);
    menu_i2.add_item(&menu_pidKP);
    menu_i2.add_item(&menu_pidKI);
    menu_i2.add_item(&menu_pidKD);
    menu_i2.add_item(&menu_pidReset);

    ms.get_root_menu().add_menu(&menu_i3);
    menu_i3.add_item(&menu_manualMode);
    menu_i3.add_item(&menu_manualServoPos);

    ms.get_root_menu().add_menu(&menu_i4);
    menu_i4.add_item(&menu_loopDelay);

    ms.display();
}

void wait_for_attachment()
{
    servo.write(SERVO_MAX_DEG);

    for (int i = 15; i > 0; --i)
    {
        if (read_LCD_buttons() == btnRIGHT)
        {
            // skip init process if btnRIGHT is pressed
            break;
        }

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("ATTACH @MAX NOW");
        lcd.setCursor(7, 1);

        lcd.print(i);
        delay(1000);
    }

    servo.write(SERVO_MIN_DEG);
    delay(2000);
}

void setTempActual(const float &fNew)
{
    fTempActual = fNew;
    menu_tempActual.set_value(fNew);
}

void setup()
{

    Serial.begin(115200);

    tempSensor.begin();
    tempSensor.requestTemperatures();
    setTempActual(tempSensor.getTempCByIndex(0));

    oPID.SetMode(AUTOMATIC);
    oPID.SetSampleTime(125); // measured loop delay

    servo.attach(SERVO_PIN);

    lcd.begin(16, 2);

    wait_for_attachment();

    setup_menu(); // real menu
}

double get_knob_pos(double fIntensity)
{
    // PID output value to servo position
    // intensity from 0 - 255
    if (fIntensity < 0 || fIntensity > 255)
    {
        Serial.println(F("Illegal intensity value"));
    }

    double fServoPos = map(fIntensity, 0, 255, SERVO_MIN_DEG, SERVO_MAX_DEG);

    return fServoPos;
}

void btn_handler()
{
    lcd_key = read_LCD_buttons();

    if (lcd_key != nLastButton)
    {
        switch (lcd_key)
        {
        case btnRIGHT:
        {
            ms.select();
            break;
        }
        case btnLEFT:
        {
            ms.back();
            break;
        }
        case btnUP:
        {
            ms.next(true);
            break;
        }
        case btnDOWN:
        {
            ms.prev(true); // loop around

            break;
        }
        case btnSELECT:
        {
            ms.select();
            break;
        }
        case btnNONE:
        {
            break;
        }
        }
        ms.display();

        nLastButton = lcd_key;
    }
}

void setServoPos(const float &fNew)
{
    fServoPos = fNew;
    menu_servoPos.set_value(fServoPos);
}

void setPidOutput(const float &fNew)
{
    // fPidOutput is changed by oPID itself
    menu_pidOut.set_value(fNew);
}

void setTempSet(const float &fNew)
{
    fTempSet = fNew;
}

void loop()
{
    // PID parameters
    oPID.SetTunings(menu_pidKP.get_value(), menu_pidKI.get_value(), menu_pidKD.get_value());
    setTempSet(menu_tempSet.get_value());

    tempSensor.requestTemperatures();
    setTempActual(tempSensor.getTempCByIndex(0));
    //setTempActual(63); //fake
    oPID.Compute();
    setPidOutput(fPidOutput);

    setServoPos(get_knob_pos(fPidOutput));
    servo.write(fServoPos);

    // Serial.print(F("Current temperature: "));
    // Serial.println(fTempActual);
    // Serial.print(F("Set temperature: "));
    // Serial.println(fTempSet);
    // Serial.print(F("PID output: "));
    // Serial.println(fPidOutput);
    // Serial.print(F("Knob pos: "));
    // Serial.println(fServoPos);

    // Serial.println(F("---"));

    btn_handler();

    menu_loopDelay.set_value(millis() - nLastLoopExecTime);
    nLastLoopExecTime = millis();

    ms.display();
}