/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 使用TWAI的示例
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-12-27 09:53:39
 */
#include <Arduino.h>
#include "HXC_TWAI.hpp"
HXC_TWAI twai(8,18,CAN_RATE_1MBIT);

//通过基类发送
esp_err_t  base_send(HXC_CAN* canbus,HXC_CAN_message_t* message){
    return canbus->send(message);
}




void test2(HXC_CAN_message_t* can_message){
    Serial.println("receive test2 data");
    //打印数据
    for (size_t i = 0; i <can_message->data_length_code ; i++)
    {
        Serial.print(can_message->data[i],HEX);
        Serial.print(",");
    }
    Serial.println();
};

void setup() {
    twai.setup();
    //通过函数添加回调
    twai.add_can_receive_callback_func(/*can地址=*/0x01,test2);

    //通过lamda添加回调
    twai.add_can_receive_callback_func(/*can地址=*/0x01,[](HXC_CAN_message_t* can_message){
        Serial.println("receive 0x01 data");
        //打印数据
        for (size_t i = 0; i <can_message->data_length_code ; i++)
        {
            Serial.print(can_message->data[i],HEX);
            Serial.print(",");
        }
        Serial.println();
    });
}

void loop() {
    //创建发送数据结构体
    HXC_CAN_message_t send_message;
    send_message.identifier=0x01;
    send_message.data_length_code=8;
    for (size_t i = 0; i < send_message.data_length_code; i++){
        send_message.data[i]=i;
    }
    auto ret=twai.send(&send_message);
    Serial.println(ret==ESP_OK?"send success":"send fail");
    delay(1000);
    auto ret2=base_send(&twai,&send_message);
    Serial.println(ret2==ESP_OK?"base send success":"base send fail");
    delay(1000);
}