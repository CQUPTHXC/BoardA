#ifndef VOFA_HPP
#define VOFA_HPP
#include "Arduino.h"
#include <map>
class VOFA_float{
    public:
    VOFA_float(HardwareSerial *serial,String _name,float default_value){
        this->name=_name;
        this->value=default_value;
        name_to_value_map[this->name]=this;
        if(!is_setup){
            xTaskCreate(read_loop,"read_loop",2048,NULL,5,NULL);
        }
        _Serial=serial;
    };
    operator float(){
        return value;
    }

    protected:
    float value;
    String name;
    static HardwareSerial* _Serial;
    static std::map<String, VOFA_float*>name_to_value_map;
    static bool is_setup;
    static void read_loop(void* p){
        if(Serial.available()>0){
            String str=_Serial->readStringUntil('\n');
            String name=str.substring(0,str.indexOf(':'));
            float value=str.substring(str.indexOf(':')+1).toFloat();
            if(name_to_value_map.find(name)!=name_to_value_map.end()){
                name_to_value_map[name]->value=value;
            }else{
                _Serial->println("VOFA_float: "+name+" not found");
            }
        }
        delay(1);
    };
};
bool VOFA_float::is_setup=false;

#endif