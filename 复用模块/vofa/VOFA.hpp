#ifndef VOFA_HPP
#define VOFA_HPP
#include "Arduino.h"
#include <map>
class VOFA_float{
    public:
    VOFA_float(String _name,float default_value){
        this->name=_name;
        this->value=default_value;
        name_to_value_map[this->name]=this;

    };
    static void setup(){
        if(!is_setup){
            xTaskCreate(read_loop,"read_loop",8192,NULL,5,NULL);
        }
    }
    operator float(){
        return value;
    }
    float read(){
        return value;
    }

    protected:
    float value;
    String name;
    static std::map<String, VOFA_float*>name_to_value_map;
    static bool is_setup;
    static void read_loop(void* p){
        while(1){
            if(Serial.available()>0){
                String str=Serial.readStringUntil('\n');
                String name=str.substring(0,str.indexOf(':'));
                float value=str.substring(str.indexOf(':')+1).toFloat();
                if(name_to_value_map.find(name)!=name_to_value_map.end()){
                    name_to_value_map[name]->value=value;
                }else{
                    Serial.println("VOFA_float: "+name+" not found");
                }
            }
            delay(1);
        }

    };
};
bool VOFA_float::is_setup=false;
std::map<String, VOFA_float*>VOFA_float::name_to_value_map;
#endif