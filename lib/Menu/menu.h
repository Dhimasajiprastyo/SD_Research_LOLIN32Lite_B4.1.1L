#ifndef _MENU_H_
#define _MENU_H_

#include <Arduino.h>

enum menuAction {
  MENU_MOVE_LEFT,
  MENU_MOVE_RIGHT,
  MENU_ENTER
};

#define MAX_ROOT_MENU     13
#define MAX_SETTING_MENU  5
#define MAX_SD_MENU       7
#define MAX_LEVEL         3
#define DEBOUNCE          300

class Menu {
  private:
    const String        rootMenu[MAX_ROOT_MENU] = {"1.Setting", "2.Analog", "3.Status", "4.Data", "5.Test", "6.Del-Rules", "7.SD menu", "8.MISC_Sensor","9.Battery safe","10.Min Signal","11.Burst Send","12.DEF.SETTINGS","13.Exit"};
    const String        settingMenu[MAX_SETTING_MENU] = {"1-1.Ambil Data", "1-2.Kirim", "1-3.Zona waktu", "1-4.Server", "1-5.Back"};
    const String        sdMenu[MAX_SD_MENU] = {"71.Disable SD", "72.Enabel SD", "73.Clear Pend", "74.Clear Log", "75.Recount Log", "76.Recount Pend", "77.Back"};
    const byte          buttonPin[3] = {34, 35, 32};
    String              router();
    int                 buttonReader();
    int                 lastButton = -1;
    int                 currentButton;
    unsigned long       debounceTimer;
    bool                buttonPressedFlag = false;

  public:
    String              dispatchMenu(menuAction action);
    int                 selector[2] = {0, 0};
    int                 levelMenu = 0;
    void                buttonBegin();
    void                reset();
    int                 buttonEvent();
};

#endif