/*
    The code in this file uses open source libraries provided by thecoderscorner

    DO NOT EDIT THIS FILE, IT WILL BE GENERATED EVERY TIME YOU USE THE UI DESIGNER
    INSTEAD EITHER PUT CODE IN YOUR SKETCH OR CREATE ANOTHER SOURCE FILE.

    All the variables you may need access to are marked extern in this file for easy
    use elsewhere.
 */

#ifndef MENU_GENERATED_CODE_H
#define MENU_GENERATED_CODE_H

#include <Arduino.h>
#include <tcMenu.h>
#include "tcMenuAdaFruitGfx.h"
#include <RuntimeMenuItem.h>
#include <IoAbstraction.h>
#include <EepromItemStorage.h>

// variables we declare that you may need to access
extern const PROGMEM ConnectorLocalInfo applicationInfo;
extern Adafruit_ST7789 gfx;
extern AdafruitDrawable gfxDrawable;
extern GraphicsDeviceRenderer renderer;

// Any externals needed by IO expanders, EEPROMs etc


// Global Menu Item exports
extern ActionMenuItem menuTurnOff;
extern BooleanMenuItem menuOptionsDemoMode;
extern BooleanMenuItem menuOptionsAmbientLightOptionsAlerts;
extern BackMenuItem menuBackOptionsAmbientLightOptions;
extern SubMenuItem menuOptionsAmbientLightOptions;
extern AnalogMenuItem menuOptionsSunscreenOptionsReminderInterval;
extern BooleanMenuItem menuOptionsSunscreenOptionsSunscreenReminder;
extern AnalogMenuItem menuOptionsSunscreenOptionsSPFLevel;
extern BackMenuItem menuBackOptionsSunscreenOptions;
extern SubMenuItem menuOptionsSunscreenOptions;
extern AnalogMenuItem menuOptionsFitzpatrickType;
extern BackMenuItem menuBackOptions;
extern SubMenuItem menuOptions;
extern TextMenuItem menuBattery;
extern BooleanMenuItem menuDismissAlert;
extern EnumMenuItem menuActiveAlert;
extern AnalogMenuItem menuMinUntilNextRmdr;
extern AnalogMenuItem menuCurrentUVIndex;

// Provide a wrapper to get hold of the root menu item and export setupMenu
inline MenuItem& rootMenuItem() { return menuCurrentUVIndex; }
void setupMenu();

// Callback functions must always include CALLBACK_FUNCTION after the return type
#define CALLBACK_FUNCTION

void CALLBACK_FUNCTION onDismissAlert(int id);
void CALLBACK_FUNCTION onFitzpatrickValueUpdate(int id);
void CALLBACK_FUNCTION onSPFUpdate(int id);
void CALLBACK_FUNCTION onSunscreenReminderToggle(int id);
void CALLBACK_FUNCTION onSunscrnRmdrIntvlChange(int id);
void CALLBACK_FUNCTION onTurnOff(int id);

#endif // MENU_GENERATED_CODE_H
