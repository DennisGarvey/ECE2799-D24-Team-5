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
#define NUM_SENSORS 4

bool unhandledPress[3] = {false, false, false};

Adafruit_MAX17048 batteryMonitor;
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

QWIICMUX mux;
Adafruit_LTR390 sensor[NUM_SENSORS];
unsigned long UVCount[NUM_SENSORS];
unsigned long ALSCount[NUM_SENSORS];
ltr390_mode_t currentSensorMode;

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
void progressBar();

void setup()
{
    Serial.begin(115200);
    
    // -------Display---------
    // turn on backlite
    pinMode(TFT_BACKLITE, OUTPUT);
    digitalWrite(TFT_BACKLITE, HIGH);
    Serial.println("backlight turned on");
    // turn on the TFT / I2C power supply
    pinMode(TFT_I2C_POWER, OUTPUT);
    digitalWrite(TFT_I2C_POWER, HIGH);
    delay(10);
    // initialize TFT
    display.init(135, 240); // Init ST7789 240x135
    display.setRotation(3);
    display.fillScreen(ST77XX_BLACK);

    display.setCursor(0,0);
    display.setTextWrap(true);
    display.setTextSize(1);
    display.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    display.println("Diplay setup sucessfull!");
    progressBar();


    display.println("Setting up buzzer...");
    // buzzer pins
    pinMode(14, OUTPUT);
    pinMode(16, OUTPUT);
    digitalWrite(14, LOW);
    display.println("Buzzer setup sucessfull!");
    progressBar();

    display.println("Setting up buttons...");
    // setup buttons inputs
    pinMode(D0, INPUT_PULLUP);
    pinMode(D1, INPUT_PULLDOWN);
    pinMode(D2, INPUT_PULLDOWN);
    // setup button interrupts
    attachInterrupt(digitalPinToInterrupt(D0), D0ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(D1), D1ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(D2), D2ISR, FALLING);
    display.println("Button setup sucessfull!");
    progressBar();

    display.println("Setting up battery monitor...");
    // battery monitor
    if (!batteryMonitor.begin())
    {
        Serial.println(F("Couldnt find Adafruit MAX17048"));
        display.println("Couldnt find Adafruit MAX17048");
        progressBar();
    }
    else
    {
        display.println("Battery monitor setup sucessful!");
        progressBar();
    }
    

    
    display.println("Setting up sensors...");
    // i2c multiplexer
    Wire.begin();
    if(mux.begin())
        display.println("Mulitplexer online!");
    else
    {
        display.println("MUX SETUP FAILED");
        display.drawRect(0, 0, 240, 135, ST77XX_RED);
        tone(16, 250);
        unsigned long error = millis();
        while(millis()<error+1000*10);
        esp_deep_sleep_start();
    }
    
    mux.setPort(0);
    progressBar();
    // init sensors
    currentSensorMode = LTR390_MODE_UVS;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        mux.setPort(i);
        sensor[i] = Adafruit_LTR390();
        if(sensor[i].begin())
        {
            sensor[i].setGain(LTR390_GAIN_18);
            sensor[i].setResolution(LTR390_RESOLUTION_20BIT);
            display.printf("Sensor %d online!\n", i);
            progressBar();
        }
        else
        {
            display.printf("SENSOR %d SETUP FAILED", i);
            display.drawRect(0, 0, 240, 135, ST77XX_RED);
            tone(16, 250);
            unsigned long error = millis();
            while(millis()<error+1000*10);
            esp_deep_sleep_start();
        }
    }
    display.println("Sensor setup sucessful!");
    progressBar();

    // WIFI AND TIME
    // WiFi.mode(WIFI_STA);
    // WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
    // configTime(GMTOFFSET, DSTOFFSET_SECONDS, NTPSERVER);

    display.println("Entering menu!");
    progressBar();
    setupMenu();
    updateUVThreshold();
}

void progressBar()
{
    static unsigned int progress = 0;
    progress++;

    display.fillRect(0, 110, progress*(240.0/10), 15, ST77XX_BLUE);
}
void loop()
{
    static unsigned long lastStatus = 0;
    if(millis()-lastStatus>100)
    {
        sendStatusOverSerial();
        lastStatus = millis();
    }
    
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

    //uv limit menu update
    double percentAccumulated = ((double)accumlutedUVCount)/accumulatedUVThreshold*100;
    if(percentAccumulated<100)
        menuOfUVLimit.setCurrentValue(percentAccumulated*100);
    else
        menuOfUVLimit.setCurrentValue(100*100);

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
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        mux.setPort(i);
        if (sensor[i].newDataAvailable())
        {
            sensor[i].setMode(currentSensorMode);
            if(currentSensorMode == LTR390_MODE_UVS)
                UVCount[i] = sensor[i].readUVS();
            else if(currentSensorMode == LTR390_MODE_ALS)
                ALSCount[i] = sensor[i].readALS();
            if(UVCount[i]==-1)
                UVCount[i] = 0;
            if(ALSCount[i]==-1)
                ALSCount[i] = 0;
            onNewData();
            lastSensorData = millis();
        }
    }
}

void onNewData()
{
    unsigned long maxUVCount = 0;
    unsigned long maxALSCount = 0;
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (maxUVCount < UVCount[i])
            maxUVCount = UVCount[i];
    }
    for (int i = 0; i < NUM_SENSORS; i++)
    {
        if (maxALSCount < ALSCount[i])
            maxALSCount = ALSCount[i];
    }
    double UVI = maxUVCount / 2300.0;
    double LUX = (0.6*maxALSCount)/(18*4);

    unsigned long deltaT = millis()-lastSensorData;

    if(menuOptionsDemoMode.getBoolean())
        maxUVCount *= 100;
    accumlutedUVCount += (deltaT/1000.0)* maxUVCount;
    menuCurrentUVIndex.setCurrentValue((uint16_t)(UVI * 10));
    menuLux.setFloatValue(LUX);
    if(LUX>50.0)
        menuLuxRecommendation.setCurrentValue(2);
    else if(LUX>5.0)
        menuLuxRecommendation.setCurrentValue(1);
    else if(LUX<=5.0)
        menuLuxRecommendation.setCurrentValue(0);
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
    static unsigned long lastStatus = 0;
    Serial.printf("Program Time: %lums (delta: %lums)\n", millis(), millis()-lastStatus);
    lastStatus = millis();
    // struct tm timeinfo;
    // if(!getLocalTime(&timeinfo))
    //     Serial.println("Failed to obtain time");
    // else
    //     Serial.println(&timeinfo, "Date/Time: %A, %B %d %Y %H:%M:%S");

    Serial.printf("Batt Voltage: %.3fV\n", batteryMonitor.cellVoltage());
    Serial.printf("Batt Percent: %.1f%%\n", batteryMonitor.cellPercent());
    Serial.printf("Batt Chg Rate: %f%%/hr\n", batteryMonitor.chargeRate());
    Serial.printf("Wifi Status: %d\n", WiFi.status());
    Serial.printf("Last Sensor Datapoint: %lums (delta: %lums)\n", lastSensorData, millis() - lastSensorData);
    Serial.printf("UVCount Data: %d %d %d %d\n", UVCount[0], UVCount[1], UVCount[2], UVCount[3]);
    Serial.printf("UVI Data: %lf %lf %lf %lf\n", UVCount[0] / 2300.0, UVCount[1] / 2300.0, UVCount[2] / 2300.0, UVCount[3] / 2300.0);
    Serial.printf("ALSCount Data: %d %d %d %d\n", ALSCount[0], ALSCount[1], ALSCount[2], ALSCount[3]);
    Serial.printf("LUX Data: %lf %lf %lf %lf\n", (0.6*ALSCount[0])/(18*4), (0.6*ALSCount[1])/(18*4), (0.6*ALSCount[2])/(18*4), (0.6*ALSCount[3])/(18*4));
    Serial.printf("UV Accumulation Threshold: ");
    print_uint64_t(accumulatedUVThreshold);
    Serial.printf("\nAccumulated UV (count seconds): ");
    print_uint64_t(accumlutedUVCount);
    Serial.printf("\n");
    Serial.printf("Percent of Threshold: %lf%%\n", ((double)accumlutedUVCount)/accumulatedUVThreshold*100);
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



void CALLBACK_FUNCTION onTurnOff(int id) 
{
    esp_deep_sleep_start();
}



void CALLBACK_FUNCTION onModeChange(int id) 
{
    switch(menuOptionsOperationalMode.getCurrentValue())
    {
        case 0: //UV
            currentSensorMode = LTR390_MODE_UVS;
            menuCurrentUVIndex.setVisible(true);
            menuOfUVLimit.setVisible(true);
            menuLux.setVisible(false);
            menuLuxRecommendation.setVisible(false);
            break;
        case 1: //ambient light
            currentSensorMode = LTR390_MODE_ALS;
            menuCurrentUVIndex.setVisible(false);
            menuOfUVLimit.setVisible(false);
            menuLux.setVisible(true);
            menuLuxRecommendation.setVisible(true);
            break;
    }
}
