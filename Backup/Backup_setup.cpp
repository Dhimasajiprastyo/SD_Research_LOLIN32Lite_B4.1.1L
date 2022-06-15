// #define _USE_STAGING_               //COMMENT = SERVER PRODUCTION

#define SOFTWARE_VERSION "B3.7.8r" //UNTUK BOARD YG SUDAH DIMODIF
#define HARDWARE_VERSION "BSA_IoT V3.5"
#define DEVICE_LOCATION "KARANGASEM 3" //SESUAIKAN DENGAN NAMA LOKASI ALAT
//#define SOFTWARE_VERSION "B3.6.8m" //UNTUK BOARD YG BELUM DIMODIF

#define DEFAULT_TIMEZONE 8 //WIB=7, WITA=8, WIT=9

#define _OFFSET_P1_ 0.0 //SESUAIKAN DENGAN NILAI OFFSET DI HARDWARE DOCS
#define _OFFSET_P2_ 0.0 //SESUAIKAN DENGAN NILAI OFFSET DI HARDWARE DOCS
#define _OFFSET_P3_ 0.0 //SESUAIKAN DENGAN NILAI OFFSET DI HARDWARE DOCS

//#define _USE_ALL_PRESSURE_          //HILANGKAN COMMENT JIKA INGIN MENGGUNAKAN 3 SENSOR PRESSURE

#ifndef _USE_ALL_PRESSURE_
  // #define _USE_PZEM_                //HILANGKAN COMMENT JIKA INGIN MENGGUNAKAN SENSOR KELISTRIKAN
  //#define _USE_SLAVE_   //TIDAK PERLU DIUBAH SEKARANG
#endif

#ifdef _USE_PZEM_
  // #define _SET_PZEM_ADDR_ //HILANGKAN KOMEN HANYA JIKA AKAN MELAKUKAN SETING ALAMAT PZEM
  #ifndef _USE_SLAVE_
    #define _DIRECT_PZEM_
  #endif
#endif