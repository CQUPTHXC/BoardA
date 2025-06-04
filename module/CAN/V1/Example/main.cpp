/*
 * @Description: 
 * @Author: qingmeijiupiao
 * @Date: 2024-07-15 22:17:54
 */
#include "ESP_CAN.hpp"


void test2(twai_message_t* can_message){
    Serial.println("receive test2 data");
    //打印数据
    for (size_t i = 0; i <can_message.data_length_code ; i++)
    {
        Serial.print(can_message.data[i],HEX);
        Serial.print(",");
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);//调试串口初始化

    //CAN模块初始化,1Mbps
    can_setup();

    //通过lamda添加回调
    add_user_can_func(/*can地址=*/0x01,[](twai_message_t* can_message){
        Serial.println("receive 0x01 data");
        //打印数据
        for (size_t i = 0; i <can_message.data_length_code ; i++)
        {
            Serial.print(can_message.data[i],HEX);
            Serial.print(",");
        }
        Serial.println();
    });

    //通过函数添加回调
    add_user_can_func(/*can地址=*/0x02,test2);

}
void loop() {
    twai_message_t send_can_message;
    send_can_message.id=0x01;//can地址
    send_can_message.extd=0;//0是标准帧，1是扩展帧
    send_can_message.self=0;//0是不自接受，1是自接受
    send_can_message.data_length_code=8;//数据长度
    send_can_message.data[0]=0x01;//数据
    send_can_message.data[1]=0x02;//数据
    send_can_message.data[2]=0x03;//数据
    send_can_message.data[3]=0x04;//数据
    send_can_message.data[4]=0x05;//数据
    send_can_message.data[5]=0x06;//数据
    send_can_message.data[6]=0x07;//数据
    send_can_message.data[7]=0x08;//数据
    auto err_code = twai_transmit(&send_can_message);//发送
    delay(500);
}
