/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 达妙电机速度位置控制
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-11 13:53:54
 */
#ifndef DM_CTRLSPEEDPOS_HPP
#define DM_CTRLSPEEDPOS_HPP
#include "DMCtrlESP.hpp"

// DMMotorSpeedPos类继承自DMMotor类，用位置速度模式控制电机
class DMMotorSpeedPos : public DMMotor {
public:
    // 构造函数，初始化电机对象，注册CAN接收回调
    DMMotorSpeedPos(HXC_CAN* can, int MST_ID, int CAN_ID);
    
    // 设置速度和位置 ：p_des，v_des 单位分别为rad和rad/s,阻尼因子必须设置为非0的正数
    void set_SpeedPos(float _speed, float _pos);

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
    float p_des = 0; // 位置，单位rad
    float v_des = 0; // 速度，单位rad/s
    TaskHandle_t ctrl_task_handle = nullptr; // 控制任务句柄
};
DMMotorSpeedPos::DMMotorSpeedPos(HXC_CAN* can, int MST_ID, int CAN_ID) : DMMotor(can, MST_ID, CAN_ID){}; 

void DMMotorSpeedPos::set_SpeedPos(float _speed, float _pos) {
    p_des = _pos;
    v_des = _speed;
}

// 设置数据交互频率，频率范围为1-1000Hz
void DMMotorSpeedPos::setDataRate(uint16_t value) {
    value = (value > 1000) ? 1000 : value;
    value = (value < 1) ? 1 : value;
    DataRateHz = value;
}

// 发送MIT电机控制数据包
void DMMotorSpeedPos::sendSpeedPospakage() {
    HXC_CAN_message_t can_message; // 定义CAN消息对象
    can_message.identifier = 0x100+this->get_CAN_ID();  // 速度位置模式控制帧为0x100+CAN_ID
    can_message.data_length_code = 8; // 数据长度为8字节
    can_message.self = false;
    memcpy(can_message.data, &p_des, 4); // 填充数据
    memcpy(can_message.data + 4, &v_des, 4);// 填充数据
    can_bus->send(&can_message); // 发送CAN数据包
}
// 参数isEnable：是否使能电机，默认为true
void DMMotorSpeedPos::setup(bool isEnable) {
    if (isEnable) {
        //电机上线检测
        while(!is_online()) {
            enable(); // 使能电机
            vTaskDelay(10/ portTICK_PERIOD_MS); // 延时等待电机上线
        }
    }

    if (ctrl_task_handle == nullptr) {
        // 创建一个控制任务，以周期性发送控制指令
        xTaskCreate(DMmotortask,DM_default_ctrl_task_name, DM_default_ctrl_task_stack_size, this, DM_default_ctrl_task_priority, &ctrl_task_handle);
    }
}

// 控制任务函数
void DMMotorSpeedPos::DMmotortask(void* _motor) {

    DMMotorSpeedPos* motor =reinterpret_cast<DMMotorSpeedPos*>(_motor);

    // 获取当前时间
    auto xLastWakeTime = xTaskGetTickCount ();
    while (1) {
        // 每次循环发送数据包
        motor->sendSpeedPospakage(); 
        // 根据数据频率设置延时
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