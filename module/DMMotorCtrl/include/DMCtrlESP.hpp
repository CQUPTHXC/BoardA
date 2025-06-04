/*
 * @LastEditors: qingmeijiupiao
 * @Description: 达妙电机控制基类
 * @Author: qingmeijiupiao
 * @LastEditTime: 2025-05-27 16:04:20
 */
#ifndef DMCtrlESP_HPP
#define DMCtrlESP_HPP
#include "HXC_CAN.hpp"
#include "DMRegister.hpp"
#include <string.h>//use memcpy memcmp
#include <math.h>
#include <FreeRTOS.h>

//#define protected public    // 开启调试模式,将所有类成员改为public


// 默认控制任务栈大小
constexpr uint32_t DM_default_ctrl_task_stack_size = 4096;
// 默认控制任务优先级
constexpr uint8_t DM_default_ctrl_task_priority = 5;
// 默认控制任务名
constexpr const char* DM_default_ctrl_task_name = "DMmotortask";

// 达妙电机基类
class DMMotor {
public:
    //禁止电机拷贝传递
    DMMotor(const DMMotor&) = delete;
    DMMotor& operator=(const DMMotor&) = delete;

    

    // 构造函数，初始化电机对象，注册CAN接收回调
    DMMotor(HXC_CAN* can, int MST_ID, int CAN_ID);

    // 使能电机 对应0xFC命令
    hxc_err_t enable();

    // 失能电机 对应0xFD命令
    hxc_err_t disable();

    // 保存电机位置零点
    hxc_err_t save_zero();

    // 清除电机错误
    hxc_err_t clear_error();

    // 获取电机的MST ID
    int get_MST_ID();

    // 获取电机的CAN ID
    int get_CAN_ID();

    // 检查电机是否在线（判断条件：20ms内未收到数据认为掉线）
    bool is_online();

    // 设置参数映射最大值
    void set_param_max(float T_max, float V_max, float P_max);

    // 设置扭矩的映射最大值
    void set_Tmax(float Tmax);

    // 获取扭矩的映射最大值
    float get_Tmax();

    // 设置回传速度的映射最大值，即上位机的Vmax
    void set_Vmax(float Vmax);
    
    // 获取回传速度的映射最大值
    float get_Vmax();

    // 设置回传位置的映射最大值，即上位机的Pmax
    void set_Pmax(float Pmax);

    // 获取回传位置的映射最大值
    float get_Pmax();

    // 获取电机的多圈位置
    int64_t get_location();

    // 重置电机的多圈位置（默认为0）
    void reset_location(int l = 0);

    // 获取电机的原始位置数据（0-65535映射到 -Pmax~Pmax）
    uint16_t get_pos_raw();

    // 获取电机的角度,单位弧度
    float get_pos_rad(bool is_multi_circle = false);

    // 获取电机的角度，单位度
    float get_pos_deg(bool is_multi_circle= false);

    // 获取电机的原始速度数据（0-4095映射到 -Vmax~Vmax）
    uint16_t get_speed_raw();

    // 获取电机的速度，单位rad/s
    float get_speed_rad();

    // 获取电机的速度，单位rpm
    float get_speed_rpm();
    
    // 获取电机的原始扭矩数据（0-4095映射到 -Tmax~Tmax）
    uint16_t get_torque_raw();

    // 获取电机的错误代码
    uint8_t get_error();

    // 获取电机的控制器温度（单位：摄氏度）
    uint8_t get_controller_temperature();

    // 获取电机的转子温度（单位：摄氏度）
    uint8_t get_motor_temperature();
    

    //can消息回调函数
    void can_message_callback(HXC_CAN_message_t* can_message);

    // 电机数据的回调函数
    void update_date_callback(uint8_t* arr);

    /**
     * @brief : 读取电机的寄存器数据
     * @return  {hxc_err_t} 成功返回ESP_OK，超时返回hxc_err_tIMEOUT
     * @Author : qingmeijiupiao
     * @param {DMRegisterAddress} addr 寄存器地址
     * @param {void*} value 读取到的数据，根据addr的类型有float或uint32_t两种类型
     */
    hxc_err_t read_register(DMRegisterAddress addr,void* value);

    // 写入电机的寄存器数据（支持写入32位值）
    hxc_err_t write_register(DMRegisterAddress addr, uint32_t value);

    // 重载写寄存器函数，支持写入浮点型数据
    hxc_err_t write_register(DMRegisterAddress addr, float value);

    //写寄存器数据立即生效，但无法进行存储，掉电后丢失，需要发送存储参数的命令，将修改的参数全部写入片内
    hxc_err_t save_register(DMRegisterAddress addr);
    
    HXC_CAN* can_bus;         // CAN总线对象

protected:



    int MST_ID;               // 电机反馈数据ID
    int CAN_ID;               // 电机控制数据ID

    uint16_t POS_raw;         // 电机位置的原始数据 16位 [0-65535]
    uint16_t VEL_raw;         // 电机速度的原始数据 12位 [0-4095]
    uint16_t TORQUE_raw;      // 电机扭矩的原始数据 12位 [0-4095]
    uint8_t MOS_temp;         // 电机控制器MOS管温度 单位：摄氏度
    uint8_t T_Rotor;          // 电机转子温度  单位：摄氏度
    uint8_t ERR;              // 电机错误代码


    float Vmax = 200; // 回传速度最大值，用于回传速度映射 rad/s
    float Pmax = 12.566; // 回传位置最大值，用于回传位置映射 rad
    float Tmax = 1; // 回传扭矩最大值，用于回传扭矩映射 Nm 暂时在控制中没有用到
    float reduction_ratio = 1; // 减速比
    uint32_t last_can_message_update_time = 0;   // 上次数据更新时间（单位：毫秒）

    int64_t location = 0;    // 多圈位置（单位：脉冲数）
    bool can_register_flag=false;//是否在等待寄存器数据
    uint8_t register_buffer_addr=0;//接收到的寄存器数据地址
    uint8_t register_buffer[4]={0,0,0,0};//寄存器数据缓冲区
    constexpr static const int READ_REGISTER_TIMEOUT=100;//读取寄存器超时时间 毫秒
};


// 构造函数：初始化电机对象，设置CAN总线和电机ID，注册回调函数
DMMotor::DMMotor(HXC_CAN* can, int MST_ID, int CAN_ID) : can_bus(can), MST_ID(MST_ID), CAN_ID(CAN_ID) {

    //设置CAN接收回调
    std::function<void(HXC_CAN_message_t*)> motor_can_message_call_back=std::bind(&DMMotor::can_message_callback,this,std::placeholders::_1);

    can->add_can_receive_callback_func(MST_ID, motor_can_message_call_back);  // 注册CAN接收回调
}

int DMMotor::get_MST_ID() {
    return MST_ID;
}

int DMMotor::get_CAN_ID() {
    return CAN_ID;
}

// 使能电机
hxc_err_t DMMotor::enable() {
    HXC_CAN_message_t can_message;
    can_message.identifier = CAN_ID;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    memset(can_message.data, 0xFF, 7);  // 数据填充0xFF
    can_message.data[7] = 0xFC;         // 设置使能命令
    return can_bus->send(&can_message); // 发送CAN命令
}

// 失能电机
hxc_err_t DMMotor::disable() {
    HXC_CAN_message_t can_message;
    can_message.identifier = CAN_ID;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    memset(can_message.data, 0xFF, 7);  // 数据填充0xFF
    can_message.data[7] = 0xFD;         // 设置失能命令
    return can_bus->send(&can_message); // 发送CAN命令
}

// 保存电机位置零点
hxc_err_t DMMotor::save_zero() {
    HXC_CAN_message_t can_message;
    can_message.identifier = CAN_ID;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    memset(can_message.data, 0xFF, 7);  // 数据填充0xFF
    can_message.data[7] = 0xFE;         // 设置零点保存命令
    return can_bus->send(&can_message); // 发送CAN命令
}

// 清除电机错误
hxc_err_t DMMotor::clear_error() {
    HXC_CAN_message_t can_message;
    can_message.identifier = CAN_ID;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    memset(can_message.data, 0xFF, 7);  // 数据填充0xFF
    can_message.data[7] = 0xFB;         // 设置清除错误命令
    return can_bus->send(&can_message); // 发送CAN命令
}

// 检查电机是否在线（判断条件：100ms内未收到数据认为掉线）
bool DMMotor::is_online() {
    if (now_time_ms() - last_can_message_update_time > 100 && last_can_message_update_time != 0) {
        return false; // 100ms内未更新数据，认为电机掉线
    }
    return true; // 电机在线
}

/**
 * @brief : 设置参数映射最大值
 * @return  {*}
 * @Author : qingmeijiupiao
 * @param {float} T_max 回传扭矩的映射最大值，即上位机的Tmax
 * @param {float} V_max 回传速度的映射最大值，即上位机的Vmax
 * @param {float} P_max 回传位置的映射最大值，即上位机的Pmax
 */
void DMMotor::set_param_max(float T_max, float V_max, float P_max){
    if(T_max>0) Tmax=T_max;
    if(V_max>0) Vmax=V_max;
    if(P_max>0) Pmax=P_max;
    
};


// 设置回传扭矩的映射最大值，即上位机的Tmax
void DMMotor::set_Tmax(float tmax) {
    Tmax = tmax;
}

// 获取回传扭矩的映射最大值
float DMMotor::get_Tmax() {
    return Tmax;
}

//设置回传速度的映射最大值，即上位机的Vmax
void DMMotor::set_Vmax(float vmax) {
    Vmax = vmax;
}

//获取回传速度的映射最大值
float DMMotor::get_Vmax() {
    return Vmax;
}

//设置回传位置的映射最大值，即上位机的Pmax
void DMMotor::set_Pmax(float pmax) {
    Pmax = pmax;
}

//获取回传位置的映射最大值
float DMMotor::get_Pmax() {
    return Pmax;
}

// 获取电机的多圈位置
int64_t DMMotor::get_location() {
    return location;
}

// 重置电机的多圈位置（默认为0）
void DMMotor::reset_location(int l) {
    location = l;
}

// 获取电机的原始位置数据（0-65535映射到 -Pmax~Pmax）
uint16_t DMMotor::get_pos_raw() {
    return POS_raw;
}


/**
 * @brief : 获取电机的角度，单位rad
 * @return {float} 角度，单位rad 单圈范围为[0,2*PI]
 * @param {bool} is_multi_circle 是否是多圈位置
 */
float DMMotor::get_pos_rad(bool is_multi_circle){
    float angle=2.f*(get_location()-32767)*Pmax/65535.f;
    if(!is_multi_circle){
        angle=std::fmod(angle,2*PI);//取模
        if(angle<0)angle+=2*PI;//如果小于0，补齐
    }
    return angle;
};


#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif


/**
 * @brief : 获取电机的角度，单位度
 * @return {float} 角度，单位度 单圈范围为[0,360]
 * @param {bool} is_multi_circle 是否是多圈位置
 */
float DMMotor::get_pos_deg(bool is_multi_circle){
    float angle=2.f*get_location()*Pmax/65535.f*180.f/PI;
    if(!is_multi_circle){
        angle=std::fmod(angle,360.f);//取模
        if(angle<0)angle+=360.f;//如果小于0，补齐
    }
    return angle;
};

// 获取电机的原始速度数据（0-4095映射到 -Vmax~Vmax）
uint16_t DMMotor::get_speed_raw() {
    return VEL_raw;
}

// 获取电机的速度，单位rad/s
float DMMotor::get_speed_rad(){
    return 2.f*(VEL_raw-2047.f)*Vmax/4095.f;
};

// 获取电机的速度，单位rpm
float DMMotor::get_speed_rpm(){
    return 2.f*(VEL_raw-2047.f)*Vmax/4095.f*60.f/(2*PI);
};

// 获取电机的原始扭矩数据（0-4095映射到 -Tmax~Tmax）
uint16_t DMMotor::get_torque_raw() {
    return TORQUE_raw;
}

// 获取目标位置


// 获取电机的错误代码
// 对应故障类型为：
// 8——超压；
// 9——欠压；
// A——过电流；
// B——MOS过温；
// C——电机线圈过温；
// D——通讯丢失；
// E——过载；
uint8_t DMMotor::get_error() {
    return ERR;
}

// 获取电机的控制器温度（单位：摄氏度）
uint8_t DMMotor::get_controller_temperature() {
    return MOS_temp;
}

// 获取电机的转子温度（单位：摄氏度）
uint8_t DMMotor::get_motor_temperature() {
    return T_Rotor;
}

void DMMotor::can_message_callback(HXC_CAN_message_t* can_message){

    if(can_register_flag){//如果是寄存器数据
        uint8_t* arr=can_message->data;
        uint16_t recived_MOTOR_CANID=arr[0]|(arr[1]<<8);
        if(recived_MOTOR_CANID==CAN_ID){//如果是本电机的数据
            can_register_flag=false;
            register_buffer_addr=arr[3];//获取寄存器地址
            memcpy(register_buffer,arr+4,4);//复制数据
        }
    }else{
        //如果是普通数据
        update_date_callback(can_message->data);
    }
}

// 更新电机数据的回调函数
void DMMotor::update_date_callback(uint8_t* arr) {
    ERR = arr[0] >> 4;              // 获取错误代码
    uint16_t POS = (arr[1] << 8) | arr[2];  // 获取位置数据
    VEL_raw = (arr[3] << 4) | (arr[4] >> 4);  // 获取速度数据
    TORQUE_raw = ((arr[4] & 0x0F) << 8) | arr[5];  // 获取扭矩数据
    MOS_temp = arr[6];              // 获取MOS管温度
    T_Rotor = arr[7];               // 获取电机转子温度

    constexpr int MAX_POSITION=(1<<16)-1;//最大位置
    constexpr int HALF_POSITION=MAX_POSITION/2;//半最大位置
    // 更新电机的多圈位置
    int delta = POS - POS_raw;
    
    // 处理位置回绕情况
    if (delta > HALF_POSITION) {
        delta -= MAX_POSITION + 1;  // 正向回绕
    } else if (delta < -HALF_POSITION) {
        delta += MAX_POSITION + 1;  // 负向回绕
    }
    
    POS_raw = POS;
    location += delta;  // 更新多圈位置
    last_can_message_update_time = now_time_ms();  // 更新最后更新时间
}

// 读取电机寄存器数据
hxc_err_t DMMotor::read_register(DMRegisterAddress addr,void* value) {
    HXC_CAN_message_t can_message;
    can_message.identifier = 0x7FF;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    can_message.data[0] = CAN_ID & 0xFF;
    can_message.data[1] = CAN_ID >> 8;
    can_message.data[2] = 0x33;
    can_message.data[3] = addr;
    hxc_err_t status = can_bus->send(&can_message); // 发送读取寄存器的CAN命令
    if(status!=HXC_OK){//发送失败
        return status;
    }
    can_register_flag=true;//等待寄存器数据
    int start_time=now_time_ms();
    while(now_time_ms()-start_time<READ_REGISTER_TIMEOUT){//等待超时
        if(register_buffer_addr==addr){
            //复制数据
            memcpy(value,register_buffer,4);
            //清除标志位
            can_register_flag=false;
            //清除数据
            register_buffer_addr=0xFF;
            memset(register_buffer,0,4);
            //返回
            return HXC_OK;
        }
        delay(1);
    }
    can_register_flag=false;
    //超时
    return HXC_ERR_TIMEOUT;
}

// 写入电机寄存器数据（支持写入32位值）
hxc_err_t DMMotor::write_register(DMRegisterAddress addr, uint32_t value) {
    HXC_CAN_message_t can_message;
    can_message.identifier = 0x7FF;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    can_message.data[0] = CAN_ID & 0xFF;
    can_message.data[1] = CAN_ID >> 8;
    can_message.data[2] = 0x55;  // 根据store参数设置命令
    can_message.data[3] = addr;
    memcpy(can_message.data + 4, &value, 4);  // 将值写入数据中
    hxc_err_t status = can_bus->send(&can_message); // 发送写入寄存器的CAN命令
    if(status!=HXC_OK){//发送失败
        return status;
    }
    can_register_flag=true;//等待寄存器数据
    int start_time=now_time_ms();
    while(now_time_ms()-start_time<READ_REGISTER_TIMEOUT){//等待超时
        if(register_buffer_addr==addr){
            //比较数据
            int temp = memcmp(register_buffer,&value,4);
            
            //清除标志位
            can_register_flag=false;
            //清除数据
            register_buffer_addr=0xFF;
            memset(register_buffer,0,4);
            //返回
            if(temp==0){
                return HXC_OK;
            }else{
                return HXC_FAIL;
            }
            
        }
        delay(1);
    }
    can_register_flag=false;
    //超时
    return HXC_ERR_TIMEOUT;
}

// 重载写寄存器函数，支持写入浮点型数据
hxc_err_t DMMotor::write_register(DMRegisterAddress addr, float value) {
    HXC_CAN_message_t can_message;
    can_message.identifier = 0x7FF;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    can_message.data[0] = CAN_ID & 0xFF;
    can_message.data[1] = CAN_ID >> 8;
    can_message.data[2] = 0x55;
    can_message.data[3] = addr;
    memcpy(can_message.data + 4, &value, 4);  // 将值写入数据中
    hxc_err_t status = can_bus->send(&can_message); // 发送写入寄存器的CAN命令
    if(status!=ESP_OK){//发送失败
        return status;
    }
    can_register_flag=true;//等待寄存器数据
    int start_time=now_time_ms();
    while(now_time_ms()-start_time<READ_REGISTER_TIMEOUT){//等待超时
        if(register_buffer_addr==addr){
            //比较数据
            int temp = memcmp(register_buffer,&value,4);
            
            //清除标志位
            can_register_flag=false;
            //清除数据
            register_buffer_addr=0xFF;
            memset(register_buffer,0,4);
            //返回
            if(temp==0){
                return HXC_OK;
            }else{
                return HXC_FAIL;
            }
            
        }
        delay(1);
    }
    can_register_flag=false;
    //超时
    return HXC_ERR_TIMEOUT;
}

hxc_err_t DMMotor::save_register(DMRegisterAddress addr){
    HXC_CAN_message_t can_message;
    can_message.identifier = 0x7FF;
    can_message.data_length_code = 8;
    can_message.extd = 0;
    can_message.self = 0;
    can_message.data[0] = CAN_ID & 0xFF;
    can_message.data[1] = CAN_ID >> 8;
    can_message.data[2] = 0xAA;
    can_message.data[3] = addr;
    return can_bus->send(&can_message); // 发送写入寄存器的CAN命令
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
