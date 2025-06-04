/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 达妙电机速度控制
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-18 14:05:55
 */
#ifndef DM_CTRLSPEED_HPP
#define DM_CTRLSPEED_HPP
#include "DMCtrlESP.hpp"

// DMMotorSpeed类继承自DMMotor类，用速度模式控制电机
class DMMotorSpeed : public DMMotor {
public:
    // 构造函数，初始化电机对象，注册CAN接收回调
    DMMotorSpeed(HXC_CAN* can, int MST_ID, int CAN_ID);
    
    // 设置速度和位置 _speed 单位为rad/s,
    void set_Speed(float _speed);

    // 设置数据交互频率，频率范围为1-1000Hz
    void setDataRate(uint16_t value);

    // 初始化位置速度控制
    // 参数isEnable：是否使能电机，默认为true
    virtual void setup(bool isEnable = true);

    /*基类接口*/

    // 使能电机 对应0xFC命令
    using DMMotor::enable;
    // 失能电机 对应0xFD命令
    using DMMotor::disable;
    // 保存电机位置零点
    using DMMotor::save_zero;
    // 清除电机错误
    using DMMotor::clear_error;
    // 获取电机的MST ID
    using DMMotor::get_MST_ID;
    // 获取电机的CAN ID
    using DMMotor::get_CAN_ID;
    // 检查电机是否在线（判断条件：20ms内未收到数据认为掉线）
    using DMMotor::is_online;
    // 获取电机的多圈位置
    using DMMotor::get_location;
    // 重置电机的多圈位置（默认为0）
    using DMMotor::reset_location;
    // 获取电机的原始位置数据（0-65535映射到 -Pmax~Pmax）
    using DMMotor::get_pos_raw;
    // 获取电机的原始速度数据（0-4095映射到 -Vmax~Vmax）
    using DMMotor::get_speed_raw;
    // 获取电机的角度,单位弧度
    using DMMotor::get_pos_rad;
    // 获取电机的角度，单位度
    using DMMotor::get_pos_deg;
    // 获取电机的速度，单位rad/s
    using DMMotor::get_speed_rad;
    // 获取电机的速度，单位rpm
    using DMMotor::get_speed_rpm;
    // 获取电机的原始扭矩数据（0-4095映射到 -Tmax~Tmax）
    using DMMotor::get_torque_raw;
    // 获取电机的错误代码
    using DMMotor::get_error;
    // 获取电机的控制器温度（单位：摄氏度）
    using DMMotor::get_controller_temperature;
    // 获取电机的转子温度（单位：摄氏度）
    using DMMotor::get_motor_temperature;

  protected:

    // 发送位置速度模式控制数据包
    void sendSpeedPospakage();

    // 控制任务函数
    static void DMmotortask(void* _motor);

    uint16_t DataRateHz = 1000; // 数据交互频率
    float v_des = 0; // 速度，单位rad/s
    TaskHandle_t ctrl_task_handle = nullptr; // 控制任务句柄
};
// 构造函数，初始化电机对象，注册CAN接收回调
DMMotorSpeed::DMMotorSpeed(HXC_CAN* can, int MST_ID, int CAN_ID) : DMMotor(can, MST_ID, CAN_ID){}; 

void DMMotorSpeed::set_Speed(float _speed) {
    v_des = _speed;
}

// 设置数据交互频率，频率范围为1-1000Hz
void DMMotorSpeed::setDataRate(uint16_t value) {
    value = (value > 1000) ? 1000 : value;
    value = (value < 1) ? 1 : value;
    DataRateHz = value;
}

// 发送MIT电机控制数据包
void DMMotorSpeed::sendSpeedPospakage() {
    HXC_CAN_message_t can_message; // 定义CAN消息对象
    can_message.identifier = 0x200+this->get_CAN_ID();  // 速度模式控制帧为0x200+CAN_ID
    can_message.data_length_code = 4; // 数据长度为8字节
    can_message.self = false;
    memcpy(can_message.data, &v_des, 4); // 填充数据
    can_bus->send(&can_message); // 发送CAN数据包
}
// 参数isEnable：是否使能电机，默认为true
void DMMotorSpeed::setup(bool isEnable) {
    if (isEnable) {
        //电机上线检测
        while(!is_online()) {
            enable(); // 使能电机
            vTaskDelay(10/ portTICK_PERIOD_MS);
        }
    }

    if (ctrl_task_handle == nullptr) {
        // 创建一个控制任务，以周期性发送控制指令
        xTaskCreate(DMmotortask,DM_default_ctrl_task_name, DM_default_ctrl_task_stack_size, this, DM_default_ctrl_task_priority, &ctrl_task_handle);
    }
}

// 控制任务函数
void DMMotorSpeed::DMmotortask(void* _motor) {
    DMMotorSpeed* motor = reinterpret_cast<DMMotorSpeed*>(_motor);
    // 获取当前时间
    auto xLastWakeTime = xTaskGetTickCount ();
    while (1) {
        motor->sendSpeedPospakage(); // 每次循环发送数据包
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / motor->DataRateHz);
    }
}
#endif
/*
                                              .=%@#=.                                               
                                            -*@@@@@@@#=.                                            
                                         .+%@@@@@@@@@@@@#=                                          
                                       -#@@@@@@@* =@@@@@@@@*:                                       
                                     =%@@@@@@@@=   -@@@@@@@@@#-                                     
                                  .+@@@@@@@@@@-     .@@@@@@@@@@%=                                   
                                .+@@@@@@@@@@@@-     +@@@@@@@@@@@@@+.                                
                               +@@@@@@@@@@@@@@@    .@@@@@@@@@@@@@@@@+.                              
                             =@@@@@@@@@@@@@@@%-     =%@@%@@@@@@@@@@@@@=                             
                           -%@@@@@@@@@@@@+..     .       -@@@@@@@@@@@@@%-                           
                         .#@@@@@@@@@@@@@#       -@+       +@@@@@@@@@@@@@@#:                         
                        +@@@@@@@@@@@@@@@@+     +@@@+     =@@@@@@@@@@@@@@@@@+                        
                      :%@@@@@@@@@@@@@@@@@+    *@@@@*     =@@@@@@@@@@@@@@@@@@%-                      
                     +@@@@@@@@@@@@@@#+*+-   .#@@@@+       :+*+*@@@@@@@@@@@@@@@*                     
                   :%@@@@@@@@@@@@@@+       :%@@@@-    .-       -@@@@@@@@@@@@@@@%:                   
                  =@@@@@@@@@@@@@@@@-      -@@@@%:    .%@+      =@@@@@@@@@@@@@@@@@=                  
                 *@@@@@@@@@@@@@@@@@@.    =@@@@#.    -@@@@+    =@@@@@@@@@@@@@@@@@@@#                 
               .%@@@@@@@@@@@@@@@@@@+    +@@@@*     =@@@@%:    .#@@@@@@@@@@@@@@@@@@@%.               
              :@@@@@@@@@@@@@@@%:::.    #@@@@+     +@@@@#        .::.*@@@@@@@@@@@@@@@@-              
             -@@@@@@@@@@@@@@@%       .%@@@@=     *@@@@*     +-       *@@@@@@@@@@@@@@@@=             
            =@@@@@@@@@@@@@@@@@#.    -@@@@@-    :%@@@@=    .#@@+     +@@@@@@@@@@@@@@@@@@=            
           =@@@@@@@@@@@@@@@@@@@:    =====.     -+===:     :====     @@@@@@@@@@@@@@@@@@@@+           
          +@@@@@@@@@@@@@@@#%%#-                                     :*%%#%@@@@@@@@@@@@@@@+          
         =@@@@@@@@@@@@@@%.       ...........................              *@@@@@@@@@@@@@@@=         
        =@@@@@@@@@@@@@@@+      .#@@@@@@@@@@@@@@@@@@@@@@@@@@#     .*:      =@@@@@@@@@@@@@@@@-        
       -@@@@@@@@@@@@@@@@@=    .%@@@@@@@@@@@@@@@@@@@@@@@@@@#     :@@@-    =@@@@@@@@@@@@@@@@@@:       
      :@@@@@@@@@@@@@@@@@%.   -@@@@%+=====================:     -@@@@%    :%@@@@@@@@@@@@@@@@@@.      
      %@@@@@@@@@@@@@=-=:    =@@@@#.                           +@@@@#.      -=--%@@@@@@@@@@@@@%      
     #@@@@@@@@@@@@@:       +@@@@*      ............. .       *@@@@*             %@@@@@@@@@@@@@+     
    =@@@@@@@@@@@@@@#.     #@@@@+     +@@@@@@@@@@@@@@@#.    .#@@@@+     +#.     +@@@@@@@@@@@@@@@:    
   .@@@@@@@@@@@@@@@@-   .%@@@@=     *@@@@@@@@@@@@@@@#     :%@@@@-     *@@%:    @@@@@@@@@@@@@@@@%    
   %@@@@@@@@@@@%%%#=   :@@@@@:    .#@@@@+-----------     -@@@@@:     #@@@@=    :#%%%@@@@@@@@@@@@*   
  =@@@@@@@@@@@=       -@@@@%.    :%@@@@-                =@@@@%.    .%@@@@=          :%@@@@@@@@@@@:  
  @@@@@@@@@@@%.      =@@@@#     -@@@@%:    .:::-:      +@@@@#     :@@@@@:    .       +@@@@@@@@@@@#  
 +@@@@@@@@@@@@@.    *@@@@*     =@@@@#.    -@@@@@:     #@@@@+     =@@@@%.    -@#     +@@@@@@@@@@@@@- 
.@@@@@@@@@@@@@#    *@%@%=     +@@@@*     =@@@@#.    .#@@@%=     +@@@@#     =@@@%.   =@@@@@@@@@@@@@% 
+@@@@@@@@*-==-                .          .           . ..       .....      .....     .=+=+@@@@@@@@@-
%@@@@@@@+                                                                                 -@@@@@@@@#
@@@@@@@-       =#%#=     -#%%#-     -#%%*.     +%%%*.    .*%%#=     :#%%#-     =%%%*.      .#@@@@@@@
@@@@@@=.::::::*@@@@@*:::-@@@@@@-:::=@@@@@%::::*@@@@@#::::%@@@@@+:---@@@@@@=---+@@@@@%------:=@@@@@@@
=@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@+
 *@@@@@@@@@@@@@@@@@@@@@@@@@@@%%##**++===----:::::------===++***##%%@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@* 
  -#@@@@@@@@@@@@@@@@%#*+=-:.                                        ..::-=+*##%@@@@@@@@@@@@@@@@@#-  
    :=*%@@@@@%#*=-:                                                             .:-=+*#%%%%##+-.    
                                                                                        
        K####      #####     ###    ###  ######.   ##########     K##    ### ###    ##W    ####W    
       #######    #######    ###    ###  ########  ##########     ###    ### ###   ###   W######    
      W###G####  ###W ####   ###    ###  ######### ##########     ###    ###  ##   ###   ###W####   
      ###   ###  ###   ###   ###    ##  ###    ###    ###         ###    ###  ### t##   ###   ###   
     G##    #   ###    ###   ##     ##  ###    ###    ###         ###    ###  ### ###   ##W         
     ###        ###    ###   ##    ###  ###    ###    ###         ##L    ##   ### ##   ###          
     ###        ###    ###  K##    ###  ###    ###    ###         ##     ##    #####   ###          
     ###       ,##     ###  ###    ###  ###   ###,    ##         G##    ###    ####    ###          
    W##        ###     ###  ###    ###  #########     ##         ##########    ####    ###          
    ###        ###     ###  ###    ###  ########     ###         ##########    ###i   K##           
    ###        ###     ###  ###    ##  #######       ###         ###    ###    ####   ###           
    ###        ###     ###  ##     ##  ###           ###         ###    ###   ##W##   ###           
    ###        ###     ##i  ##    ###  ###           ###         ###    ##    ## ##   ###           
    ###        ###    ###  ,##    ###  ###           ###         ##     ##   ### ##   ###           
    ###    ### ###    ###  K##    ###  ###           ##         t##    ###   ##  ###  ###    ###    
    ###   G##i ###   ###   .##   ###.  ##t           ##         ###    ###  ###  ###  W##,   ###    
     ########  W##W#####    ########   ##           ###         ###    ###  ##    ##   ####W###     
     #######    #######     #######   ###           ###         ###    ### ###    ##.  #######      
      #####      #####       #####    ###           ###         ###    ### ##W    ###   #####       
                   ###                                                                              
                   ###                                                                              
                   #####                                                                            
                    ####                                                                            
                      K                                                                             
*/