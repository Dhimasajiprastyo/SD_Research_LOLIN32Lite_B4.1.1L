#ifndef _SDop_
#define _SDop_

#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"

class SDOP{
    public:
        void            listAllDir(fs::FS &fs, const char * dirname, uint8_t levels);
        //uint8_t         listFile_pending(fs::FS &fs, const char * dirname, char *array_pending, uint8_t total_file_pending);
        int16_t         countFile(fs::FS &fs, const char * dirname, uint8_t limit);
        void            createDir(fs::FS &fs, const char * path);
        void            removeDir(fs::FS &fs, const char * path);
        String          readFile(fs::FS &fs, const char * path);
        bool            writeFile(fs::FS &fs, const char * path, const char * message);
        void            appendFile(fs::FS &fs, const char * path, const char * message);
        bool            renameFile(fs::FS &fs, const char * path1, const char * path2);
        bool            deleteFile(fs::FS &fs, const char * path);
        uint16_t        deleteManyFile(fs::FS &fs, const char * dirname, uint16_t number);
        void            testFileIO(fs::FS &fs, const char * path);
        String          cardInfo();


};

#endif

