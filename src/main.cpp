// Tone generation
// not yet implemented
#define TONE_PIN 3

// DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

#define DS18B20_PIN 2
#define DS18B20_RESOLUTION 12 // 9-12
#define DS18B20_WAIT 750 / (1 << (12 - DS18B20_RESOLUTION))

OneWire oneWire(DS18B20_PIN);
DallasTemperature tempSensor(&oneWire);

// EEPROM stuff
#include <EEPROM.h>
#define EEPROM_ADDR_KP 0
#define EEPROM_ADDR_KI 0 + 1 * sizeof(double)
#define EEPROM_ADDR_KD 0 + 2 * sizeof(double)
#define EEPROM_ADDR_TEMP_SET 0 + 3 * sizeof(double)

// PI controller
#include <ArduPID.h>

double fTempSet = 70, fTempActual;
double fPidOutput; // 0-PID_MAX_VALUE

// PID controller parameters - before being read from EEPROM
#define PID_kP_default 1.4
#define PID_kI_default 0
#define PID_kD_default 0

#define PID_T 100         //PID cycle time in ms, target loop delay
#define PID_MAX_VALUE 100 // pid output is between 0 and this

PID_IC oPID(&fPidOutput, PID_kP_default, PID_kI_default, PID_kD_default, 100, PID_T); // with clamping as anti windup

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

    void render_numeric_display_menu_item(NumericDisplayMenuItem<float> const &menu_item) const
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

void setPidOutput(const float &fNew);
void resetPidState()
{
    setPidOutput(0.0);
    oPID.SetSaturation(0, PID_MAX_VALUE);
    oPID.Reset();
}

void onPidReset(MenuComponent *comp)
{
    resetPidState();
}

void onTempSetChange(MenuComponent *comp);
void onPidParamChange(MenuComponent *comp);

void onSaveToEeprom(MenuComponent *comp);
void onReadFromEeprom(MenuComponent *comp);

DefaultRenderer my_renderer;

// Menu variables
MenuSystem ms(my_renderer);

Menu menu_i1("Temperature [dC]");
NumericDisplayMenuItem<float> menu_tempActual("Actual", NULL, NULL, 23.2);
NumericMenuItem menu_tempSet("Set", &onTempSetChange, 69, 0, 99, 0.5, NULL);

Menu menu_i2("PID values");
NumericDisplayMenuItem<float> menu_pidOut("PID out", NULL, NULL, 999);
NumericDisplayMenuItem<float> menu_curDutyCycle("DutyCyc", NULL, &MenuHelpers::format_int, 0);
NumericMenuItem menu_pidKP("k_P", &onPidParamChange, PID_kP_default, 0, 10, 0.025, NULL);
NumericMenuItem menu_pidKI("k_I", &onPidParamChange, PID_kI_default, 0, 10, 0.001, NULL);
NumericMenuItem menu_pidKD("k_D", &onPidParamChange, PID_kD_default, 0, 10, 0.001, NULL);
MenuItem menu_pidReset("Reset PID state", &onPidReset);

Menu menu_i3("Manual mode");
NumericMenuItem menu_manualMode("Manual (0/1)", NULL, 0, 0, 1, 1.0, &MenuHelpers::format_int);
NumericMenuItem menu_manualPower("Power", NULL, 0, 0, PID_MAX_VALUE, 10, NULL);

Menu menu_i4("Misc debug");
NumericDisplayMenuItem<float> menu_loopDelay("LoopLag", NULL, &MenuHelpers::format_int, -1);
MenuItem menu_readFromEeprom("Read from EEPROM", &onReadFromEeprom);
MenuItem menu_saveToEeprom("Save to EEPROM", &onSaveToEeprom);

void onTempSetChange(MenuComponent *comp = NULL)
{
    fTempSet = menu_tempSet.get_value();
}

void onPidParamChange(MenuComponent *comp = NULL)
{
    oPID.SetTunings(menu_pidKP.get_value(), menu_pidKI.get_value(), menu_pidKD.get_value(), 100);
}

void showShortMsg(const char *msg1, const char *msg2, uint16_t delayTime = 1000)
{
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.write(msg1);
    lcd.setCursor(0, 1);
    lcd.write(msg2);
    lcd.display();
    delay(delayTime);
}

void onSaveToEeprom(MenuComponent *comp)
{
    EEPROM.put(EEPROM_ADDR_TEMP_SET, menu_tempSet.get_value());
    EEPROM.put(EEPROM_ADDR_KP, menu_pidKP.get_value());
    EEPROM.put(EEPROM_ADDR_KI, menu_pidKI.get_value());
    EEPROM.put(EEPROM_ADDR_KD, menu_pidKD.get_value());
    showShortMsg("<Success>", "Saved to EEPROM");
}

void onReadFromEeprom(MenuComponent *comp)
{
    double kp, ki, kd, tempSet;
    EEPROM.get(EEPROM_ADDR_KP, kp);
    EEPROM.get(EEPROM_ADDR_KI, ki);
    EEPROM.get(EEPROM_ADDR_KD, kd);
    EEPROM.get(EEPROM_ADDR_TEMP_SET, tempSet);
    menu_pidKP.set_value(kp);
    menu_pidKI.set_value(ki);
    menu_pidKD.set_value(kd);
    onPidParamChange();
    fTempSet = tempSet;
    menu_tempSet.set_value(tempSet);
    onTempSetChange();

    showShortMsg("<Success>", "Read EEPROM");
}

// Heater control with relay
#define RELAIS_PIN A3
bool bHeaterOn = false;
#define PWM_CYCLE_LENGTH 10000
unsigned long nPwmTimeOnCurrentCycle = 0 * PWM_CYCLE_LENGTH; // time heater should be on
unsigned long nPwmBeginThisCycle = 0;
double fPwmNextDutyCycle = 0;                                     // intensity of next cycle
#define PWM_MIN_DUTY_CYCLE_ABOVE_0 (double)500 / PWM_CYCLE_LENGTH // either duty cycle of 0 or this value, so that motor does not have to turn again after only e.g. 50 ms

unsigned long nLastLoopExecTime = 0, nLastPidExecTime = 0, nLastTempRequestTime = 0, nLastDisplayRefreshTime = 0;

void setup_menu()
{
    lcd.clear();

    ms.get_root_menu().add_menu(&menu_i1);
    menu_i1.add_item(&menu_tempActual);
    menu_i1.add_item(&menu_tempSet);

    ms.get_root_menu().add_menu(&menu_i2);
    menu_i2.add_item(&menu_pidOut);
    menu_i2.add_item(&menu_curDutyCycle);
    menu_i2.add_item(&menu_pidKP);
    menu_i2.add_item(&menu_pidKI);
    menu_i2.add_item(&menu_pidKD);
    menu_i2.add_item(&menu_pidReset);

    ms.get_root_menu().add_menu(&menu_i3);
    menu_i3.add_item(&menu_manualMode);
    menu_i3.add_item(&menu_manualPower);

    ms.get_root_menu().add_menu(&menu_i4);
    menu_i4.add_item(&menu_loopDelay);
    menu_i4.add_item(&menu_readFromEeprom);
    menu_i4.add_item(&menu_saveToEeprom);

    ms.display();
    nLastDisplayRefreshTime = millis();
}

void message_on_boot()
{
    unsigned long nStartTime = millis();
    while(millis() < nStartTime+3000)
    {
        if (read_LCD_buttons() == btnRIGHT)
        {
            // skip init process if btnRIGHT is pressed
            break;
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("GLUEHWOSTAT v2.0"));
        lcd.setCursor(7, 1);
        lcd.print((int)((nStartTime+3000-millis())/1000));
        delay(200); // so display does not blink
    }
}

void setTempActual(const float &fNew)
{
    fTempActual = fNew;
    menu_tempActual.set_value(fNew);
}

void setup()
{
    Serial.begin(115200);

    fTempSet = menu_tempSet.get_value();

    tempSensor.begin();
    tempSensor.setResolution(DS18B20_RESOLUTION);
    tempSensor.setWaitForConversion(false);

    tempSensor.requestTemperatures();
    nLastTempRequestTime = millis();

    resetPidState();

    pinMode(RELAIS_PIN, OUTPUT);
    digitalWrite(RELAIS_PIN, LOW);

    pinMode(TONE_PIN, OUTPUT);

    lcd.begin(16, 2);

    message_on_boot();

    setup_menu(); // real menu

    // wait manually this time for temp reading if necessary
    while (millis() - nLastTempRequestTime < DS18B20_WAIT)
    {
    }
    setTempActual(tempSensor.getTempCByIndex(0));

    // request again but do not wait
    tempSensor.requestTemperatures();
    nLastTempRequestTime = millis();
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
            //ms.select();
            ms.reset();
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

void setPidOutput(const float &fNew)
{
    fPidOutput = fNew;
    menu_pidOut.set_value(fNew);
}

void loop()
{
    unsigned long nNow = millis();
    unsigned long nLoopDelay = nNow - nLastLoopExecTime;

    if (nNow - nLastTempRequestTime > DS18B20_WAIT)
    {
        setTempActual(tempSensor.getTempCByIndex(0));
        tempSensor.requestTemperatures();
        nLastTempRequestTime = millis(); // request again
    }

    if ((int)menu_manualMode.get_value() == 0)
    {
        // automatic

        if ((nNow - nLastPidExecTime) >= PID_T)
        {
            // only execute if enough time has passed
            nLastPidExecTime = nNow;

            double fCurrentErr = fTempSet - fTempActual;
            oPID.Compute(fCurrentErr);
            setPidOutput(fPidOutput);

            fPwmNextDutyCycle = fPidOutput / PID_MAX_VALUE;
        }
    }
    else
    {
        // manual mode
        fPwmNextDutyCycle = menu_manualPower.get_value()/PID_MAX_VALUE;
    }

    // range check fPwmNextDutyCycle
    if (fPwmNextDutyCycle < PWM_MIN_DUTY_CYCLE_ABOVE_0 && fPwmNextDutyCycle > 0)
    {
        // round
        if (fPwmNextDutyCycle >= PWM_MIN_DUTY_CYCLE_ABOVE_0 / 2)
        {
            fPwmNextDutyCycle = PWM_MIN_DUTY_CYCLE_ABOVE_0;
        }
        else
        {
            fPwmNextDutyCycle = 0;
        }
    }
    // TODO: PWM_MAX_DUTY_CYCLE_BELOW_MAX

    // check whether to turn heater off
    if (bHeaterOn)
    {
        if (nNow - nPwmBeginThisCycle > nPwmTimeOnCurrentCycle)
        {
            // turn off
            digitalWrite(RELAIS_PIN, LOW);
            bHeaterOn = false;
        }
        // else wait until ON cycle is over
    }

    // check for cycle end
    if (nNow - nPwmBeginThisCycle > PWM_CYCLE_LENGTH)
    {
        // turn on to begin new cycle
        nPwmBeginThisCycle = nNow;
        nPwmTimeOnCurrentCycle = (fPwmNextDutyCycle * PWM_CYCLE_LENGTH);
        menu_curDutyCycle.set_value(nPwmTimeOnCurrentCycle);
        Serial.println(fPwmNextDutyCycle);
        Serial.println(nPwmTimeOnCurrentCycle);
        if (nPwmTimeOnCurrentCycle > 0)
        {
            digitalWrite(RELAIS_PIN, HIGH);
            bHeaterOn = true;
        }
    }

    btn_handler();

    menu_loopDelay.set_value(nLoopDelay);
    nLastLoopExecTime = nNow;

    if (nNow - nLastDisplayRefreshTime > 200)
    {
        ms.display();
        nLastDisplayRefreshTime = nNow;
    }
}
