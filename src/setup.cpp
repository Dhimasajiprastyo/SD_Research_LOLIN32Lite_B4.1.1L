#pragma region 1
  #define ONLY_PRESSURE 0
  #define PRESSURE_AND_ELECTRICITY 1
  #define PRESSURE_AND_ULTRASONIC_FLOW 2
  #define ONLY_WATER_LEVEL 3
  #define USE_SLAVE 4
  #define PRESSURE_AND_ELECTRICITY_AND_ULTRASONIC_FLOW 5
#pragma endregion 1

//---------------------SETUP PARAMETER------------------------------------------------------
// #define SOFTWARE_VERSION "B3.7.9b" //UNTUK BOARD YG BELUM DIMODIF
// #define SOFTWARE_VERSION      "B4.0.1d" //UNTUK BOARD YG SUDAH DIMODIF
#define SOFTWARE_VERSION      "B4.1.1g" //UNTUK BOARD YG SUDAH DIMODIF
#define HARDWARE_VERSION      "BSA_IoT V4"

#define DEVICE_LOCATION       "Testing" //SESUAIKAN DENGAN NAMA LOKASI ALAT
#define DEFAULT_TIMEZONE      8 //WIB=7, WITA=8, WIT=9
#define LCD_PRESSURE_OFFSET   0 //SESUAIKAN DENGAN NILAI OFFSET DI HARDWARE DOCS

// #define _USE_STAGING_
#define _SEND_DATA_ON_HARD_RESET_
// #define _DYNAMIC_SERIAL_PORT_PIN_  //jika pin serial komunikasi antara kelistrikan dan flow dibedakan
// #define _USE_ALL_PRESSURE_
// #define _USE_FLOW_ID_5_
// #define _USE_FLOW_ID_6_
#define _ADDON_               ONLY_PRESSURE 
/**SELECT 1:
 * ONLY_PRESSURE
 * PRESSURE_AND_ELECTRICITY
 * PRESSURE_AND_ULTRASONIC_FLOW
 * PRESSURE_AND_ELECTRICITY_AND_ULTRASONIC_FLOW
 * ONLY_WATER_LEVEL
 * USE_SLAVE
 */
//------------------END SETUP PARAMETER-----------------------------------------------------

#pragma region 
  #if _ADDON_ == PRESSURE_AND_ELECTRICITY
    #define _USE_PZEM_
    #define _SET_PZEM_ADDR_
    #define _ACTIVATE_SENSOR_SERIAL1_
    #define _SET_ELECTRIC_ICON_
    // #define _DIRECT_PZEM_
  #elif _ADDON_ == ONLY_WATER_LEVEL
    #define _USE_A1_FOR_WATER_LEVEL_
    #define _FIND_AWLR_VOLTAGE_
  #elif _ADDON_ == PRESSURE_AND_ULTRASONIC_FLOW
    #define _ACTIVATE_SENSOR_SERIAL1_
    #define _USE_ULTRASONIC_FLOW_METER_
    // #define _DIRECT_ULTRASONIC_FLOW_METER_
  #elif _ADDON_ == PRESSURE_AND_ELECTRICITY_AND_ULTRASONIC_FLOW
    #define _ACTIVATE_SENSOR_SERIAL1_
    #define _USE_PZEM_
    #define _SET_PZEM_ADDR_
    #define _USE_ULTRASONIC_FLOW_METER_
    #define _SET_FLOW_ELECTRIC_ICON_
  #endif

  #define _OFFSET_P1_ LCD_PRESSURE_OFFSET 
  #define _OFFSET_P2_ LCD_PRESSURE_OFFSET 
  #define _OFFSET_P3_ LCD_PRESSURE_OFFSET 
#pragma endregion