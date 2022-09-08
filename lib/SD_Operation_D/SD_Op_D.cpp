#include "SD_Op_D.h"

void SDOP::listAllDir(fs::FS &fs, const char * dirname, uint8_t levels){
    Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDirectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listAllDir(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}


/* uint8_t SDOP::listFile_pending(fs::FS &fs, const char * dirname, char array_pending[], uint8_t total_file_pending){
    Serial.printf("Listing pending files: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return 0;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return 0;
    }
    
    uint8_t i=0;
    File file = root.openNextFile(); 
    while(file && (i<total_file_pending)){
        if(!file.isDirectory()){
            Serial.print("  FILE: ");
            Serial.print(file.name());
            // Serial.print("  SIZE: ");
            // Serial.println(file.size());
            snprintf(array_pending[i],31,"%s",file.name());
            Serial.println("_____");
            Serial.println(file_list[i]);
            Serial.println("=====");
            i++;
        } 
        file = root.openNextFile();
    }
    return total_file;
}
 */
 
int16_t SDOP::countFile(fs::FS &fs, const char * dirname, uint16_t limit){
    //Serial.printf("Listing directory: %s\n", dirname);

    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return -1;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return -1;
    }
    
    uint16_t total_file=0;
    File file = root.openNextFile();
    //hitung jumlah file yang ada di dalam folder------------
    if(limit>0){
        while(file){ //jika penghitungan limit file ditampilkan
            if(!file.isDirectory()){
                total_file++;
                if(total_file > limit){
                    //total_file --;
                    Serial.println("Count LIMITED!");
                    break;
                }
            }
            file = root.openNextFile();
        }
    }else{
        while(file){ //tidak ada limit jumlah
            if(!file.isDirectory()){
                total_file++;
            }
            file = root.openNextFile();
        }
    }
    
    //char file_list[total_file][31];
    Serial.print("=-=FilePending : ");
    if(limit>0){
        if(total_file>=limit){
            Serial.printf("%i+\n",total_file);
        }else{
            Serial.println(total_file);
        }
        
    }else{
        Serial.println(total_file);
    }
    //Serial.println("=-=-=-=-=-=-=-=");
    root.close();
    return total_file;
}

void SDOP::createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s --> ", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

void SDOP::removeDir(fs::FS &fs, const char * path){
    Serial.printf("Removing Dir: %s --> ", path);
    if(fs.rmdir(path)){
        Serial.println("Dir removed");
    } else {
        Serial.println("rmdir failed");
    }
}

String SDOP::readFile(fs::FS &fs, const char * path, bool print){
    Serial.printf("Reading file: %s --> ", path);
    String out;
    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        String failread; failread.reserve(30);
        failread = "{\"ans\":\"0\"}";
        file.close();
        return failread;
    }else{

    //Serial.print("Read from file: ");
    out = file.readString();
    /* while(file.available()){
        Serial.write(file.read());
    } */
    if(print){
        Serial.println(out);
    }else{
        Serial.println("EXIST!");
    }
    file.close();
    return out;
    }
    //return out;
}

bool SDOP::writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s --> ", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return false;
    }
    if(file.print(message)){
        //Serial.println("File written");
        Serial.println("OK");
        return true;
    } else {
        //Serial.println("Write failed");
        Serial.println("FAILED");
        return false;
    }
    file.close();
}

void SDOP::appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

bool SDOP::renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("Renamed!");
        return true;
    } else {
        Serial.println("Rename FAILED!");
        return false;
    }
}

bool SDOP::deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s --> ", path);
    if(fs.remove(path)){
        Serial.println("OK");
        return true;
    } else {
        Serial.println("FAILED");
        return false;
    }
}

uint16_t SDOP::deleteManyFile(fs::FS &fs, const char * dirname, uint16_t number){
    File root = fs.open(dirname);
    if(!root){
        Serial.println("Failed to open directory");
        return 0;
    }
    if(!root.isDirectory()){
        Serial.println("Not a directory");
        return 0;
    }
    uint16_t deleted=0;
    File file = root.openNextFile();
    while(file){
        if(!file.isDirectory()){
            if(fs.remove(file.name())){            
                Serial.println("OK");
                deleted++;
            } else {
                Serial.println("FAILED");
            }
        }
        if((deleted<number)||(number==0)){
            file = root.openNextFile();
        }else{
            Serial.println("Done!");
            break;
        }
    }    
    Serial.printf("%i Deleted!\n",deleted);
    root.close();
    file.close();
    return deleted;
}

void SDOP::testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

String SDOP::cardInfo(){
    uint8_t cardType = SD.cardType();
    String SDtype = "???";
    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return "NO SD card!";
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
        SDtype = "MMC";
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
        SDtype = "SDSC";
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
        SDtype = "SDHC";
    } else {
        Serial.println("UNKNOWN");
        SDtype = "UNKNW";
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    //Serial.printf("Total space: %lluKB\n", SD.totalBytes() / 1024);
    Serial.printf("Used space: %lluKB\n", SD.usedBytes()/1024);
    return SDtype;
}