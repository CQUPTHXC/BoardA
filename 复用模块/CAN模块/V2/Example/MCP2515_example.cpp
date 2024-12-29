/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 使用MCP2515的示例
 * @Author: qingmeijiupiao
 * @LastEditTime: 2024-12-29 18:39:31
 */
#include <Arduino.h>
#include "HXC_MCP2515.hpp"
SPIClass spi;
HXC_MCP2515 CAN_BUS(&spi,/*CS=*/38,/*INT=*/14);

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
    //设置spi时钟 MCP2515的标称最大时钟为10MHZ 实测40MHZ也可以正常工作
    spi.setFrequency(1e7);
    
    //设置spi引脚
    spi.begin(/*SCK=*/21,/*MISO=*/48,/*MOSI*/47);
    
    //初始化并设置can总线速率
    CAN_BUS.setup(CAN_RATE_1MBIT);

    //通过函数添加回调
    CAN_BUS.add_can_receive_callback_func(/*can地址=*/0x01,test2);

    //通过lamda添加回调
    CAN_BUS.add_can_receive_callback_func(/*can地址=*/0x01,[](HXC_CAN_message_t* can_message){
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
    send_message.identifier=0x01;//can地址
    send_message.extd=false;//是否扩展帧
    send_message.data_length_code=8;//数据长度

    for (size_t i = 0; i < send_message.data_length_code; i++){
        send_message.data[i]=i;//数据
    }
    //发送数据
    auto ret=CAN_BUS.send(&send_message);
    
    Serial.println(ret==ESP_OK?"send success":"send fail");
    delay(1000);
    auto ret2=base_send(&CAN_BUS,&send_message);
    Serial.println(ret2==ESP_OK?"base send success":"base send fail");
    delay(1000);
}