#include "menu.h"

void Menu::buttonBegin() {
  pinMode(buttonPin[0], INPUT);
  pinMode(buttonPin[1], INPUT);
  pinMode(buttonPin[2], INPUT);
}

int Menu::buttonReader() {
  if (digitalRead(buttonPin[0])){
    return MENU_MOVE_LEFT;
  } else if (digitalRead(buttonPin[1])) {
    return MENU_ENTER;
  } else if (digitalRead(buttonPin[2])) {
    return MENU_MOVE_RIGHT;
  } else {
    lastButton = -1;
    return -1;
  }
}

int Menu::buttonEvent() {
  int buttonRead = buttonReader();
  if(buttonRead > -1){
    if(millis() - debounceTimer > DEBOUNCE) {
      if(lastButton != buttonRead){
        debounceTimer = millis();
        lastButton = buttonRead;
        return buttonRead;
      } else {
        return -1;
      }
    } else {
      return -1;
    }
  } else {
    return -1;
  }
}


String Menu::router() {
  uint8_t max_second_menu = 0;
  // selector[0] = selector[0] >= MAX_ROOT_MENU ? MAX_ROOT_MENU - 1 : selector[0];
  // selector[0] = selector[0] < 0 ? 0 : selector[0];
  selector[0] = selector[0] >= MAX_ROOT_MENU ? 0 : selector[0];
  selector[0] = selector[0] < 0 ? MAX_ROOT_MENU - 1 : selector[0];
  // selector[1] = selector[1] >= MAX_SETTING_MENU ? MAX_SETTING_MENU - 1 : selector[1];
  // selector[1] = selector[1] < 0 ? 0 : selector[1];

  if(selector[0]==0){
    max_second_menu = MAX_SETTING_MENU;
  } else if(selector[0]==6){
    max_second_menu = MAX_SD_MENU;
  }

  selector[1] = selector[1] >= max_second_menu ? 0 : selector[1];
  selector[1] = selector[1] < 0 ? max_second_menu - 1 : selector[1];

  if(levelMenu == 0){
    return rootMenu[selector[0]];
  } else if (levelMenu == 1 && selector[0] == 0) {
    return settingMenu[selector[1]];
  } else if (levelMenu == 1 && selector[0] == 1) {
    return "#analog$";
  } else if (levelMenu == 1 && selector[0] == 2) {
    return "#status$";
  } else if (levelMenu == 1 && selector[0] == 3) {
    return "#data$";
  } else if (levelMenu == 1 && selector[0] == 4) {
    return "#test$";
  } else if (levelMenu == 1 && selector[0] == 5) {
    return "#delrules$";
  } else if (levelMenu == 1 && selector[0] == 6) {
    return sdMenu[selector[1]];
  } else if (levelMenu == 1 && selector[0] == 7) {
    return "#pzemset$";
  } else if (levelMenu == 1 && selector[0] == 8) {
    return "#bat$";
  } else if (levelMenu == 1 && selector[0] == 9) {
    return "#sig$";
  } else if (levelMenu == 1 && selector[0] == 10) {
    return "#burst$";
  } else if (levelMenu == 1 && selector[0] == 11) {
    return "#defset$";
  } else if (levelMenu == 1 && selector[0] == 12) {
    return "#exit$";
    //-----------------------------------SETTINGS-------------------------------
  } else if (levelMenu == 2 && selector[0] == 0 && selector[1] == 0) {
    return "#ambil_data$";
  } else if (levelMenu == 2 && selector[0] == 0 && selector[1] == 1) {
    return "#kirim_data$";
  } else if (levelMenu == 2 && selector[0] == 0 && selector[1] == 2) {
    return "#timezone$";
  } else if (levelMenu == 2 && selector[0] == 0 && selector[1] == 3) {
    return "#server$";
  } else if (levelMenu == 2 && selector[0] == 0 && selector[1] == 4) {
    return "#back$";
    //-----------------------------------END SETTINGS-------------------------------
  } else if (levelMenu == 2 && selector[0] == 6 && selector[1] == 0) {
    return "#disablesd$";
  } else if (levelMenu == 2 && selector[0] == 6 && selector[1] == 1) {
    return "#enablesd$";
  } else if (levelMenu == 2 && selector[0] == 6 && selector[1] == 2) {
    return "#clrpend$";
  } else if (levelMenu == 2 && selector[0] == 6 && selector[1] == 3) {
    return "#clrsent$";
  } else if (levelMenu == 2 && selector[0] == 6 && selector[1] == 4) {
    return "#rcntsent$";
  } else if (levelMenu == 2 && selector[0] == 6 && selector[1] == 5) {
    return "#rcntpend$";
  } else if (levelMenu == 2 && selector[0] == 6 && selector[1] == 6) {
    return "#back$";
    //-----------------------------------END SD-------------------------------
  } else {
    return "undefined";
  }
}

void Menu::reset() {
  levelMenu = 0;
  selector[0] = 0;
  selector[1] = 0;
}

String Menu::dispatchMenu(menuAction action) {
  switch (action)
  {
  case MENU_MOVE_LEFT: selector[levelMenu] -= 1; break;
  case MENU_MOVE_RIGHT: selector[levelMenu] += 1; break;
  case MENU_ENTER: levelMenu += 1; break;
  default:
    break;
  }
  return levelMenu >= MAX_LEVEL ? "break" : router();
}

