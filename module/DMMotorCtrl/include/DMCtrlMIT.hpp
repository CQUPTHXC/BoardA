/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 达妙电机MIT控制
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-04-13 15:11:50
 */
#ifndef DM_Ctrl_MIT_HPP
#define DM_Ctrl_MIT_HPP

#include "DMCtrlESP.hpp"

// DMMotorMIT类继承自DMMotor类，用MIT模式控制电机
class DMMotorMIT : public DMMotor {
public:
    // 构造函数1: 通过CAN总线初始化电机
    DMMotorMIT(HXC_CAN* can, int MST_ID, int CAN_ID);

    // 构造函数2: 通过CAN总线初始化电机，并设置控制参数Kp和Kd
    DMMotorMIT(HXC_CAN* can, int MST_ID, int CAN_ID, uint8_t _kd, uint8_t _kp);

    // 设置数据交互频率，频率范围为1-1000Hz
    void setDataRate(uint16_t value);

    // 设置力矩控制参数（TFF），取值范围为[-1,1]
    void set_tff(uint16_t __tff);

    // 设置位置模式控制参数
    // 参数p_des：16位整数，0-65535，映射到[-P_max, P_max]
    void set_pdes(uint16_t _p_des);

    // 设置速度模式控制参数
    // 参数v_des：12位整数，范围为[-V_max, V_max]
    void set_vdes(uint16_t _v_des);

    // 初始化MIT电机控制
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
    //设置参数映射最大值
    using DMMotor::set_param_max;
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


    // 发送MIT电机控制数据包
    void sendMITpakage();

    // 控制任务函数
    static void DMmotortask(void* _motor);
    TaskHandle_t ctrl_task_handle = nullptr; // 控制任务句柄

    uint8_t Kp = 0; // 位置控制系数
    uint8_t Kd = 0; // 速度控制系数
    uint16_t DataRateHz = 1000; // 数据交互频率,单位为Hz,默认为1000
    uint16_t p_des = 0; // 位置串级控制参数
    uint16_t v_des = 0.5 * (1 << 12); // 速度串级控制参数
    uint16_t t_ff = 0.5 * (1 << 12); // 力矩串级控制参数


};

// 构造函数1: 通过CAN总线初始化电机
DMMotorMIT::DMMotorMIT(HXC_CAN* can, int MST_ID, int CAN_ID) 
    : DMMotor(can, MST_ID, CAN_ID) {

    }

// 构造函数2: 通过CAN总线初始化电机，并设置控制参数Kp和Kd
DMMotorMIT::DMMotorMIT(HXC_CAN* can, int MST_ID, int CAN_ID, uint8_t _kd, uint8_t _kp)
    : DMMotor(can, MST_ID, CAN_ID), Kd(_kd), Kp(_kp) {}

// 设置数据交互频率，频率范围为1-1000Hz
void DMMotorMIT::setDataRate(uint16_t value) {
    value = (value > 1000) ? 1000 : value;
    value = (value < 1) ? 1 : value;
    DataRateHz = value;
}

// 设置力矩控制参数（TFF）取值范围为[0,4095]映射到[-T_max, T_max]
void DMMotorMIT::set_tff(uint16_t __tff) {
    Kp = 0;  // 当使用力矩控制时，Kp应为0
    Kd = 0;  // 当使用力矩控制时，Kd应为0
    t_ff = (__tff > ((1 << 12) - 1)) ? ((1 << 12) - 1) : __tff; // 限制t_ff的最大值
}

// 设置位置模式控制参数
// 参数p_des：16位整数，0-65535，映射到[-P_max, P_max]
void DMMotorMIT::set_pdes(uint16_t _p_des) {
    p_des = (_p_des > ((1 << 16) - 1)) ? ((1 << 16) - 1) : _p_des; // 限制p_des的最大值
}

// 设置速度模式控制参数
// 参数v_des：12位整数，取值范围为[0,4095]映射到[-V_max, V_max]
void DMMotorMIT::set_vdes(uint16_t _v_des) {
    v_des = (_v_des > ((1 << 12) - 1)) ? ((1 << 12) - 1) : _v_des; // 限制v_des的最大值
}

// 初始化MIT电机控制
// 参数isEnable：是否使能电机，默认为true
void DMMotorMIT::setup(bool isEnable) {
    if (isEnable) {
        // 此处可以进行电机上线检测代码，例如：
        // while(!is_online()) {
        //     enable(); // 使能电机
        //     delay(10); // 延时等待电机上线
        // }
    }

    if (ctrl_task_handle == nullptr) {
        // 创建一个控制任务，以周期性发送控制指令
        xTaskCreate(DMmotortask, DM_default_ctrl_task_name, DM_default_ctrl_task_stack_size, this, DM_default_ctrl_task_priority, &ctrl_task_handle);
    }
}

// 发送MIT电机控制数据包
void DMMotorMIT::sendMITpakage() {
    uint8_t arr[8];
    arr[0] = p_des >> 8;       // 高字节
    arr[1] = p_des & 0xFF;     // 低字节
    arr[2] = v_des >> 4;       // 高4位
    arr[3] = ((v_des & 0x0F) << 4) | ((Kp >> 8) & 0x0F); // v_des低4位和Kp高4位组合
    arr[4] = Kp & 0xFF;        // Kp低8位
    arr[5] = Kd >> 4;          // Kd高4位
    arr[6] = ((Kd & 0x0F) << 4) | ((t_ff >> 8) & 0x0F); // Kd低4位和t_ff高4位组合
    arr[7] = t_ff & 0xFF;      // t_ff低8位

    HXC_CAN_message_t can_message; // 定义CAN消息对象
    can_message.identifier = this->get_CAN_ID();  // 电机ID就是CAN地址
    can_message.data_length_code = 8; // 数据长度为8字节
    can_message.self = false;
    memcpy(can_message.data, arr, 8); // 填充数据
    can_bus->send(&can_message); // 发送CAN数据包
}

// 控制任务函数
void DMMotorMIT::DMmotortask(void* _motor) {
    DMMotorMIT* motor = reinterpret_cast<DMMotorMIT*>(_motor);
    // 获取当前时间
    auto xLastWakeTime = xTaskGetTickCount();
    while (1) {
        // 每次循环发送数据包
        motor->sendMITpakage();
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