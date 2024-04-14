#include <Arduino.h>
#include <WiFi.h>
#include <Time.h>
#include <Wire.h>
#include <Adafruit_MAX1704X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_LTR390.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include "config.h"
#include "ECE2799-D24 Team 5_menu.h"

bool unhandledPress[3] = {false, false, false};

Adafruit_MAX17048 batteryMonitor;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

QWIICMUX mux;
Adafruit_LTR390 sensor[3];
unsigned long UVCount[3];
unsigned long ALSCount[3];

unsigned long nextSunscreenReminder = -1;
unsigned long lastSensorData = 0;
uint64_t accumlutedUVCount = 1; // unit of "Count Seconds"
uint64_t accumulatedUVThreshold = -1;

void D0ISR();
void D1ISR();
void D2ISR();
bool buttonWasPressed(BUTTON button);
void sendStatusOverSerial();
void refreshSensorData();
void onNewData();
void changeSunscreenInterval(int minutes);
void checkSunscreenReminderAlert();
void checkUVAlert();
void updateUVThreshold();
void print_uint64_t(uint64_t num);

void setup()
{
    Serial.begin(115200);

    // buzzer pins
    pinMode(14, OUTPUT);
    pinMode(16, OUTPUT);
    digitalWrite(14, LOW);

    // setup buttons inputs
    pinMode(D0, INPUT_PULLUP);
    pinMode(D1, INPUT_PULLDOWN);
    pinMode(D2, INPUT_PULLDOWN);
    // setup button interrupts
    attachInterrupt(digitalPinToInterrupt(D0), D0ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(D1), D1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(D2), D2ISR, FALLING);

    // battery monitor
    if (!batteryMonitor.begin())
    {
        Serial.println(F("Couldnt find Adafruit MAX17048"));
    }
    // -------Display---------
    // turn on backlite
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);

    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);
    // initialize TFT
    display.init(135, 240); // Init ST7789 240x135
    display.setRotation(3);
    display.fillScreen(ST77XX_BLACK);
    Serial.println(F("Display Initialized Sucessfully"));

    // i2c multiplexer
    Wire.begin();
    mux.begin();
    mux.setPort(0);
    // init sensors
    for (int i = 0; i < 3; i++)
    {
        mux.setPort(i);
        sensor[i] = Adafruit_LTR390();
        sensor[i].begin();
        sensor[i].setGain(LTR390_GAIN_18);
        sensor[i].setResolution(LTR390_RESOLUTION_20BIT);
    }

    // WIFI AND TIME
    // WiFi.mode(WIFI_STA);
    // WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
    // configTime(GMTOFFSET, DSTOFFSET_SECONDS, NTPSERVER);

    setupMenu();
    updateUVThreshold();
}
void loop()
{
    if (millis() % 100 == 0)
        sendStatusOverSerial();
    
    refreshSensorData();
    checkUVAlert();

    // remap button with inverse logic
    pinMode(17, OUTPUT);
    digitalWrite(17, !digitalRead(0));
    pinMode(17, INPUT);

    // battery percentage update
    char buffer[5];
    dtostrf(batteryMonitor.cellPercent(), 1, 1, buffer);
    menuBattery.setTextValue(buffer);

    // menu loop
    taskManager.runLoop();

    // check if sunscreen threshold has passed
    checkSunscreenReminderAlert();
    if (menuOptionsSunscreenOptionsSunscreenReminder.getBoolean()) // update next reminder on main menu
    {
        menuMinUntilNextRmdr.setCurrentValue((nextSunscreenReminder - millis()) / (100 * 60));
    }
    // buzzer siren functionality
    if (menuActiveAlert.getCurrentValue() > 0)
    {
        static bool flipflop = false;
        static unsigned long lastFlip = millis();
        if (lastFlip + 250 < millis())
        {
            flipflop = !flipflop;
            tone(16, flipflop ? 1000 : 1250);
            lastFlip = millis();
        }
    }
}

void refreshSensorData()
{
    for (int i = 0; i < 3; i++)
    {
        mux.setPort(i);
        if (sensor[i].newDataAvailable())
        {
            // sensor[i].setMode(LTR390_MODE_ALS);
            // ALSCount[i]=sensor[i].readALS();
            sensor[i].setMode(LTR390_MODE_UVS);
            UVCount[i] = sensor[i].readUVS();
            onNewData();
            lastSensorData = millis();
        }
    }
}

void onNewData()
{
    unsigned long maxUVCount = 0;
    unsigned long maxALSCount = 0;
    for (int i = 0; i < 3; i++)
    {
        if (maxUVCount < UVCount[i])
            maxUVCount = UVCount[i];
    }
    for (int i = 0; i < 3; i++)
    {
        if (maxALSCount < ALSCount[i])
            maxALSCount = ALSCount[i];
    }
    double UVI = maxUVCount / 2300.0;

    unsigned long deltaT = millis()-lastSensorData;

    if(menuOptionsDemoMode.getBoolean())
        maxUVCount *= 100;
    accumlutedUVCount += (deltaT/1000.0)* maxUVCount;
    menuCurrentUVIndex.setCurrentValue((uint16_t)(UVI * 10));
}

bool buttonWasPressed(BUTTON button)
{
    bool returnVal = unhandledPress[button];
    unhandledPress[button] = false;
    return returnVal;
}
void D0ISR()
{
    // menuActiveAlert.setCurrentValue(1); //test alert
    static unsigned long lastInterrupt = 0;
    unsigned long thisInterrupt = millis();

    if (lastInterrupt - thisInterrupt > DEBOUNCEDELAY)
    {
        unhandledPress[D0] = true;
    }
    lastInterrupt = thisInterrupt;
}
void D1ISR()
{
    static unsigned long lastInterrupt = 0;
    unsigned long thisInterrupt = millis();

    if (lastInterrupt - thisInterrupt > DEBOUNCEDELAY)
    {
        unhandledPress[D1] = true;
    }
    lastInterrupt = thisInterrupt;
}
void D2ISR()
{
    static unsigned long lastInterrupt = 0;
    unsigned long thisInterrupt = millis();

    if (lastInterrupt - thisInterrupt > DEBOUNCEDELAY)
    {
        unhandledPress[D2] = true;
    }
    lastInterrupt = thisInterrupt;
}

void sendStatusOverSerial()
{
    Serial.println("---------------------------------");
    if(menuOptionsDemoMode.getBoolean())
        Serial.printf(":::::DEMO MODE ACTIVE:::::\n");
    Serial.print("Program Time: ");
    Serial.print(millis());
    Serial.println("ms");
    // struct tm timeinfo;
    // if(!getLocalTime(&timeinfo))
    //     Serial.println("Failed to obtain time");
    // else
    //     Serial.println(&timeinfo, "Date/Time: %A, %B %d %Y %H:%M:%S");

    Serial.printf("Batt Voltage: %.3fV\n", batteryMonitor.cellVoltage());
    Serial.printf("Batt Percent: %.1f%%\n", batteryMonitor.cellPercent());
    Serial.printf("Batt Chg Rate: %f%%/hr\n", batteryMonitor.chargeRate());
    Serial.printf("Wifi Status: %d\n", WiFi.status());
    Serial.printf("Last Sensor Datapoint: %lu, delta: %lu\n", lastSensorData, millis() - lastSensorData);
    Serial.printf("UVCount Data: %d %d %d\n", UVCount[0], UVCount[1], UVCount[2]);
    Serial.printf("UVI Data: %lf %lf %lf\n", UVCount[0] / 2300.0, UVCount[1] / 2300.0, UVCount[2] / 2300.0);
    Serial.printf("ALSCount Data: %d %d %d\n", ALSCount[0], ALSCount[1], ALSCount[2]);
    Serial.printf("UV Accumulation Threshold: ");
    print_uint64_t(accumulatedUVThreshold);
    Serial.printf("\nAccumulated UV (count seconds): ");
    print_uint64_t(accumlutedUVCount);
    Serial.printf("\n");
    Serial.printf("Percent of Threshold: %lf\n", ((double)accumlutedUVCount)/accumulatedUVThreshold*100);
    Serial.printf("Seconds till sunscreen reminder: %d\n", (nextSunscreenReminder - millis()) / 1000);
    Serial.println("---------------------------------");
}

void CALLBACK_FUNCTION onSunscrnRmdrIntvlChange(int id)
{
    changeSunscreenInterval(menuOptionsSunscreenOptionsReminderInterval.getCurrentValue() + 10);
}

void changeSunscreenInterval(int minutes)
{
    if(menuOptionsDemoMode.getBoolean())
        minutes /= 10;
    nextSunscreenReminder = millis() + (minutes * (unsigned long)(60 * 1000));
}
void checkSunscreenReminderAlert()
{
    if (menuOptionsSunscreenOptionsSunscreenReminder.getBoolean() && millis() > nextSunscreenReminder)
    {
        menuActiveAlert.setCurrentValue(3);
    }
}

void checkUVAlert()
{
    if(accumlutedUVCount>accumulatedUVThreshold)
    {
        menuActiveAlert.setCurrentValue(1);
    }
}

void CALLBACK_FUNCTION onDismissAlert(int id)
{
    if (menuDismissAlert.getBoolean())
    {
        switch(menuActiveAlert.getCurrentValue())
        {
            case 0: //no alert
                break;
            case 1: //uv exposure alert
                accumlutedUVCount = 0;
                break;
            case 2: //ambient light
                break;
            case 3: //reapply sunscreen
                changeSunscreenInterval(menuOptionsSunscreenOptionsReminderInterval.getCurrentValue() + 10);
                break;
        }
        menuActiveAlert.setCurrentValue(0);
        noTone(16);
        menuDismissAlert.setBoolean(false);
    }
}

void CALLBACK_FUNCTION onSunscreenReminderToggle(int id)
{
    menuMinUntilNextRmdr.setVisible(menuOptionsSunscreenOptionsSunscreenReminder.getBoolean());
    changeSunscreenInterval(menuOptionsSunscreenOptionsReminderInterval.getCurrentValue() + 10);
}

void updateUVThreshold()
{
    int fpv = menuOptionsFitzpatrickType.getCurrentValue() + 1;
    uint64_t spf = menuOptionsSunscreenOptionsSPFLevel.getCurrentValue();
    spf = spf==0 ? 1 : spf;

    switch (fpv)
    {
    case 1:
        accumulatedUVThreshold = (spf*154127-535)*60;
        break;
    case 2:
        accumulatedUVThreshold = (spf*230027-535)*60;
        break;
    case 3:
        accumulatedUVThreshold = (spf*459928+4634)*60;
        break;
    case 4:
        accumulatedUVThreshold = (spf*690021-2655)*60;
        break;
    case 5:
        accumulatedUVThreshold = (spf*919889+2708)*60;
        break;
    case 6:
        accumulatedUVThreshold = (spf*1194938+2044)*60;
        break;
    }
}

void CALLBACK_FUNCTION onFitzpatrickValueUpdate(int id)
{
    updateUVThreshold();
}

void CALLBACK_FUNCTION onSPFUpdate(int id)
{
    updateUVThreshold();
}

void print_uint64_t(uint64_t num)
{
    char rev[128];
    char *p = rev + 1;

    while (num > 0)
    {
        *p++ = '0' + (num % 10);
        num /= 10;
    }
    p--;
    /*Print the number which is now in reverse*/
    while (p > rev)
    {
        Serial.print(*p--);
    }
}
