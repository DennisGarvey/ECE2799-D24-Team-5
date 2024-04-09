// #include <Arduino.h>
// #include <WiFi.h>
// #include <Time.h>
// #include <Wire.h>
// #include <Adafruit_MAX1704X.h>
// #include <Adafruit_GFX.h>
// #include <Adafruit_ST7789.h>
// #include <Adafruit_LTR390.h>
// #include <SparkFun_I2C_Mux_Arduino_Library.h>
// #include "config.h"
// #include "ECE2799-D24 Team 5_menu.h"

// typedef enum {ACTIVESTANDBY, ALERT, MENU, LPM} STATE;
// STATE currentState;
// STATE previousState;

// bool unhandledPress[3] = {false, false, false};


// Adafruit_MAX17048 batteryMonitor;
// Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// QWIICMUX mux; 
// Adafruit_LTR390 sensor[3];
// unsigned long UVCount[3];
// unsigned long ALSCount[3];

// void D0ISR();
// void D1ISR();
// void D2ISR();
// void changeState(STATE newState);
// bool buttonWasPressed(BUTTON button);
// void sendStatusOverSerial();
// void refreshSensorData();
// void onNewData();

// void setup()
// {
//     Serial.begin(115200);

//     //buzzer pins
//     pinMode(14, OUTPUT);
//     pinMode(16, OUTPUT);
//     digitalWrite(14, LOW);

//     //setup buttons inputs
//     pinMode(D0, INPUT_PULLUP);
//     pinMode(D1, INPUT_PULLDOWN);
//     pinMode(D2, INPUT_PULLDOWN);
//     //setup button interrupts
//     attachInterrupt(digitalPinToInterrupt(D0), D0ISR, RISING);
//     attachInterrupt(digitalPinToInterrupt(D1), D1ISR, FALLING);
//     attachInterrupt(digitalPinToInterrupt(D2), D2ISR, FALLING);

//     //battery monitor
//     if (!batteryMonitor.begin()) {Serial.println(F("Couldnt find Adafruit MAX17048"));}
//     // -------Display---------
//     // turn on backlite
//     pinMode(TFT_BACKLITE, OUTPUT);
//     digitalWrite(TFT_BACKLITE, HIGH);

//     // turn on the TFT / I2C power supply
//     pinMode(TFT_I2C_POWER, OUTPUT);
//     digitalWrite(TFT_I2C_POWER, HIGH);
//     delay(10);
//     // initialize TFT
//     display.init(135, 240); // Init ST7789 240x135
//     display.setRotation(3);
//     display.fillScreen(ST77XX_BLACK);
//     Serial.println(F("Display Initialized Sucessfully"));

//     //i2c multiplexer
//     Wire.begin();
//     mux.begin();
//     mux.setPort(0);
//     //init sensors
//     for(int i = 0; i<3; i++)
//     {
//         mux.setPort(i);
//         sensor[i] = Adafruit_LTR390();
//         sensor[i].begin();
//         sensor[i].setGain(LTR390_GAIN_18);
//         sensor[i].setResolution(LTR390_RESOLUTION_20BIT);
//     }
    

//    //WIFI AND TIME
//     //WiFi.mode(WIFI_STA);
//     //WiFi.begin(WLAN_SSID, WLAN_PASSWORD);
//     //configTime(GMTOFFSET, DSTOFFSET_SECONDS, NTPSERVER);
//     changeState(ACTIVESTANDBY);

//     setupMenu();
// }
// void loop()
// {
//     sendStatusOverSerial();
//     refreshSensorData();

//     //remap button with inverse logic
//     pinMode(17, OUTPUT);
//     digitalWrite(17, !digitalRead(0));
//     pinMode(17, INPUT);

//     //battery percentage update 
//     char buffer[5];
//     dtostrf(batteryMonitor.cellPercent(), 1, 1, buffer);
//     menuBattery.setTextValue(buffer);

//     //menu loop
//     taskManager.runLoop();

//     //dismiss alert functionality
//     if(menuDismissAlert.getBoolean())
//     {
//         menuActiveAlert.setCurrentValue(0);
//         menuDismissAlert.setBoolean(false);
//         noTone(16);
//     }

//     //buzzer siren functionality
//     if(menuActiveAlert.getCurrentValue()>0)
//     {
//         static bool flipflop = false;
//         static unsigned long lastFlip = millis();
//         if(lastFlip+250<millis())
//         {
//             flipflop = !flipflop;
//             tone(16, flipflop ? 1000 : 1250);
//             lastFlip = millis();
//         }
//     }
    
// }

// void changeState(STATE newState)
// {
//     switch(newState)
//     {
//         case ACTIVESTANDBY:
//             break;
//         case MENU:
//             break;
//         case ALERT:
//             break;
//     }
//     previousState = currentState;
//     currentState = newState;
// }

// void refreshSensorData()
// {
//     for(int i = 0; i < 3; i++)
//     {
//         mux.setPort(i);
//         if(sensor[i].newDataAvailable())
//         {
//             sensor[i].setMode(LTR390_MODE_ALS);
//             ALSCount[i]=sensor[i].readALS();
//             sensor[i].setMode(LTR390_MODE_UVS);
//             UVCount[i]=sensor[i].readUVS();
//             onNewData();
//         }
//     }
// }

// void onNewData()
// {
//     unsigned long maxUVCount = 0;
//     unsigned long maxALSCount = 0;
//     for(int i = 0; i<3; i++)
//     {
//         if(maxUVCount<UVCount[i])
//             maxUVCount = UVCount[i];
//     }
//     for(int i = 0; i<3; i++)
//     {
//         if(maxALSCount<ALSCount[i])
//             maxALSCount = ALSCount[i];
//     }
//     double UVI = maxUVCount/2300.0;
//     menuCurrentUVIndex.setCurrentValue((uint16_t) UVI);
//     menuAmbientLight.setCurrentValue((uint16_t) maxALSCount);
// }
// void drawActiveMode(unsigned int encoderValue, RenderPressMode clicked)
// {
//     display.drawRect(0,0, 240, 135, ST77XX_GREEN);
//     display.drawRect(50, 50, 50, 50, ST77XX_WHITE);
//     display.setTextWrap(false);
//     display.setTextColor(ST77XX_BLUE, ST77XX_BLACK);
//     display.setTextSize(1);
    
//     //time
//     display.drawRect(5,5, 160, 25, ST77XX_WHITE);
//     struct tm timeinfo;
//     //getLocalTime(&timeinfo);
//     display.setCursor(10, 10);
//     display.println(&timeinfo, "%A, %B %d %Y");
//     display.setCursor(10, 20);
//     display.println(&timeinfo, "%H:%M:%S");
//     //battery percent
//     display.setCursor(175, 10);
//     display.printf("Bat:%.1f%%", batteryMonitor.cellPercent());

// }

// bool buttonWasPressed(BUTTON button)
// {
//     bool returnVal = unhandledPress[button];
//     unhandledPress[button] = false;
//     return returnVal;
// }
// void D0ISR() 
// {
//     menuActiveAlert.setCurrentValue(1);
//     static unsigned long lastInterrupt = 0;
//     unsigned long thisInterrupt = millis();

//     if(lastInterrupt-thisInterrupt > DEBOUNCEDELAY)
//     {
//         unhandledPress[D0] = true;
//     }
//     lastInterrupt = thisInterrupt;
// }
// void D1ISR() 
// {
//     static unsigned long lastInterrupt = 0;
//     unsigned long thisInterrupt = millis();

//     if(lastInterrupt-thisInterrupt > DEBOUNCEDELAY)
//     {
//         unhandledPress[D1] = true;
//     }
//     lastInterrupt = thisInterrupt;
// }
// void D2ISR() 
// {
//     static unsigned long lastInterrupt = 0;
//     unsigned long thisInterrupt = millis();

//     if(lastInterrupt-thisInterrupt > DEBOUNCEDELAY)
//     {
//         unhandledPress[D2] = true;
//     }
//     lastInterrupt = thisInterrupt;
// }

// void sendStatusOverSerial()
// {
//     Serial.println("---------------------------------");
//     Serial.print("Program Time: "); Serial.print(millis()); Serial.println("ms");
//     // struct tm timeinfo;
//     // if(!getLocalTime(&timeinfo))
//     //     Serial.println("Failed to obtain time");
//     // else
//     //     Serial.println(&timeinfo, "Date/Time: %A, %B %d %Y %H:%M:%S");

//     Serial.printf("Current State: %d\n", currentState);
//     Serial.printf("Previous State: "); Serial.println(previousState);
//     Serial.printf("Batt Voltage: %.3fV\n", batteryMonitor.cellVoltage());
//     Serial.printf("Batt Percent: %.1f%%\n", batteryMonitor.cellPercent());
//     Serial.printf("Batt Chg Rate: %f%%/hr\n", batteryMonitor.chargeRate());
//     Serial.printf("Wifi Status: %d\n", WiFi.status());
//     Serial.printf("UVCount Data: %d %d %d\n", UVCount[0], UVCount[1], UVCount[2]);
//     Serial.printf("UVI Data: %lf %lf %lf\n", UVCount[0]/2300.0, UVCount[1]/2300.0, UVCount[2]/2300.0);
//     Serial.printf("ALSCount Data: %d %d %d\n", ALSCount[0], ALSCount[1], ALSCount[2]);
//     Serial.println("---------------------------------");
// }