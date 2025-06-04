/*
 * @LastEditors: qingmeijiupiao
 * @Description: HXC达妙电机控制，基于MIT控制
 * @Author: qingmeijiupiao
 * @LastEditTime: 2025-05-11 13:54:23
 */
#ifndef HXC_DMCtrl_HPP
#define HXC_DMCtrl_HPP
#include "DMCtrlMIT.hpp"
#include "PID_CONTROL.hpp" //PID控制器和now_time_us()

class HXC_DMCtrl : protected DMMotorMIT{
    public:
    
    /**
     * @description: 
     * @return {*}
     * @Author: qingmeijiupiao
     * @param {HXC_CAN*} can
     * @param {int} MST_ID 
     * @param {int} CAN_ID
     */
    HXC_DMCtrl(HXC_CAN*can,int MST_ID,int CAN_ID);

    HXC_DMCtrl(HXC_CAN*can,int MST_ID,int CAN_ID,pid_param speed_pid,pid_param location_pid);
    ~HXC_DMCtrl(){};
    //初始化,位置闭环,使能
    void setup(bool is_enable = true)override;

    // 停止电机，并根据需要卸载使能
    void stop(bool need_unload = true);

    // 设置位置闭环控制参数
    void set_location_pid(float _location_Kp = 0, float _location_Ki = 0, float _location_Kd = 0, float __dead_zone = 0, float _max_speed = 0);
    
    // 设置位置闭环控制参数（传入pid_param结构体）
    void set_location_pid(pid_param pid);

    // 设置速度闭环控制参数
    void set_speed_pid(float _speed_Kp = 0, float _speed_Ki = 0, float _speed_Kd = 0, float __dead_zone = 0, float _max_curunt = 0);

    // 设置速度闭环控制参数（传入pid_param结构体）
    void set_speed_pid(pid_param pid);

    // 设置多圈目标位置,单位为16位编码器值，0-65535映射为-PMAX~PMAX
    void set_taget_pos_location(int64_t _location);
    
    // 设置多圈目标位置,单位为弧度
    void set_taget_pos_rad(float rad);

    // 设置多圈目标位置,单位为度
    void set_taget_pos_deg(float deg);

    // 设置最大力矩（0-1）
    void set_max_Torque(float _max_Torque);

    // 卸载使能
    void unload();

    // 使能电机
    void load();

    // 获取电机是否使能
    bool get_is_load();

    // 设置目标速度，单位：RPM，加速度控制
    virtual void set_speed(float speed, float acce = 0);

    // 获取目标位置
    int64_t get_location_target();

    // 获取转子目标速度
    float get_taget_speed();

    // 设置电机加速度，单位：RPM/s
    void set_acceleration(float acce = 0);

    // 获取减速箱减速比
    float get_reduction_ratio();

    // 设置速度环位置误差系数，可以理解为转子与预期选择位置每相差一圈加 speed_location_K RPM速度补偿
    void set_speed_location_K(float _K = 1000);

    // 获取闭环控制频率
    int get_dynamic_send_frequency();

    // 设置闭环控制频率 当min!=max时，控制频率为min到max之间的自动变化频率 值范围1~1000Hz
    void set_control_frequency(uint16_t min = 100, uint16_t max = 1000);

    /**
     * @description: 当电机的输出电流需要根据位置进行映射的时候，可以传入位置到电流映射函数的指针
     * 例如电机做摇臂控制电机时，电流需要做三角函数映射，那么可以传入经过三角函数的位置到电流映射函数的指针
     * @return {*}
     * @author: qingmeijiupiao
     * @date: 2024-04-23 14:00:29
     * @param 返回值为力矩 -1~1，输入为位置(int64_t)的映射函数或者lambda
     */
    void add_location_to_Torque_func(std::function<int(int64_t)> func);

    /*基类接口*/

    //设置控制频率，也影响CAN接发消息的的频率
    using DMMotorMIT::setDataRate;
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
    // 检查电机是否在线（判断条件：100ms内未收到数据认为掉线）
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

    int64_t location_taget = 0;        // 目标位置,回传的编码器值(16bit)作为单位 例如PMAX=12.566 那么旋转一圈为(2*PI/PMAX)*65535=32768
    int64_t speed_location_taget = 0;  // 速度目标位置,这里为了最大化精度,以回传的位置数据作为单位,0-65535映射为0-PMAX
    pid_param default_location_pid_parmater={0.047,0.092,0,50,500};  // //默认参数

    PID_CONTROL location_pid_contraler;      // 位置PID控制器
    pid_param default_speed_pid_parmater={0.001,0.0006,0,1,1};//默认参数

    PID_CONTROL speed_pid_contraler;         // 速度PID控制器
    
    float max_Torque = 0.8; // 最大扭矩 0-1
    float taget_speed = 0;     // 目标速度,单位RPM



    float acceleration = 0;    // 电机加速度（单位：rad/s²）
    int speed_location_K = 1000;   // 速度环位置误差系数 可以理解为转子与预期选择位置每相差一圈加 speed_location_K RPM速度补偿
    int control_frequency_max = 1000;  // 控制频率最大值（单位：Hz）
    int control_frequency_min = 100;      // 控制频率最小值（单位：Hz）
    uint32_t send_package_num = 0; // 已经发送包的数量
    float dynamic_send_frequency = 0; // 动态发送频率

    /*用于速度计算的减速比，因为DM3519的位置映射的是电机屁股，
    而回传速度却是输出轴的速度，需要乘以减速比才能得到电机屁股目标位置
    其他电机用不到这个变量
    */
    float speed_reduction_ratio = 1;  

    TaskHandle_t torque_func_handle = nullptr;    // 力矩控制任务句柄
    TaskHandle_t speed_func_handle = nullptr;     // 速度闭环控制任务句柄
    TaskHandle_t location_func_handle = nullptr;  // 位置闭环控制任务句柄

    // 力矩控制任务的优先级
    constexpr static int torque_task_Priority = 5;
    // 力矩控制任务的堆栈大小  
    constexpr static int torque_task_stack_size = 4096;
    // 力矩控制更新任务
    static void torque_ctrl_task(void* p);

    // 速度控制任务的优先级
    constexpr static int speed_task_Priority = 5;  
    // 速度控制任务的堆栈大小
    constexpr static int speed_task_stack_size = 4096;
    // 速度闭环控制任务
    static void speed_contral_task(void* n);

    // 位置控制任务的优先级
    constexpr static int location_task_Priority = 5;  
    // 位置控制任务的堆栈大小
    constexpr static int location_task_stack_size = 4096;
    // 位置闭环控制任务
    static void location_contral_task(void* n);

    // 位置到力矩的映射函数 ，输入为位置，输出为力矩[-1~1]，需要在函数中限幅
    std::function<float(int64_t)> location_to_Torque_func = [](int64_t location) { return 0; };

};


HXC_DMCtrl::HXC_DMCtrl(HXC_CAN*can,int MST_ID,int CAN_ID):DMMotorMIT(can,MST_ID,CAN_ID){
    set_speed_pid(default_speed_pid_parmater);
    set_location_pid(default_location_pid_parmater);
};

HXC_DMCtrl::HXC_DMCtrl(HXC_CAN*can,int MST_ID,int CAN_ID,pid_param speed_pid,pid_param location_pid):DMMotorMIT(can,MST_ID,CAN_ID){
    set_speed_pid(speed_pid);
    set_location_pid(location_pid);
}

//初始化,位置闭环,使能
void HXC_DMCtrl::setup(bool is_enable){
    if(torque_func_handle==nullptr){
        xTaskCreate(torque_ctrl_task, "torque_ctrl_task", torque_task_stack_size, this, torque_task_Priority, &torque_func_handle);
    };
    
    if (is_enable&&speed_func_handle==nullptr) {
        vTaskDelay(200/ portTICK_PERIOD_MS);
        enable(); // 使能电机
        xTaskCreate(speed_contral_task, "speed_contral_task", speed_task_stack_size, this, speed_task_Priority, &speed_func_handle);  
    }

    //获取四个关键参数
    read_register(Gr,&reduction_ratio);//减速比
    read_register(VMAX,&Vmax);//回传速度映射范围
    read_register(PMAX,&Pmax);//回传位置映射范围
    read_register(TMAX,&Tmax);//回传扭矩映射范围

    vTaskDelay(100/ portTICK_PERIOD_MS);//防止连续读取寄存器导致数据错误

}


// 停止电机，并根据需要卸载使能
void HXC_DMCtrl::stop(bool need_unload){
    set_speed(0);
    if (need_unload) {
        unload();
    }
}

// 设置位置闭环控制参数
void HXC_DMCtrl::set_location_pid(float _location_Kp, float _location_Ki, float _location_Kd, float __dead_zone, float _max_speed){
    location_pid_contraler.setPram(_location_Kp,_location_Ki,_location_Kd,__dead_zone,_max_speed);
}
// 设置位置闭环控制参数（传入pid_param结构体）
void HXC_DMCtrl::set_location_pid(pid_param pid){
    location_pid_contraler.setPram(pid);
}

// 设置速度闭环控制参数
void HXC_DMCtrl::set_speed_pid(float _speed_Kp, float _speed_Ki, float _speed_Kd, float __dead_zone, float _max_curunt){
    speed_pid_contraler.setPram(_speed_Kp,_speed_Ki,_speed_Kd,__dead_zone,_max_curunt);
}

// 设置速度闭环控制参数（传入pid_param结构体）
void HXC_DMCtrl::set_speed_pid(pid_param pid){
    speed_pid_contraler.setPram(pid);
}

// 设置多圈目标位置
void HXC_DMCtrl::set_taget_pos_location(int64_t _location){
    location_taget = _location;
    if (location_func_handle == nullptr) {
        xTaskCreate(location_contral_task, "location_contral_task", location_task_stack_size, this, location_task_Priority, &location_func_handle);
    }
}

// 设置多圈目标位置,单位为弧度
void HXC_DMCtrl::set_taget_pos_rad(float rad){
    int64_t location = (int64_t)((rad/(2.f*Pmax))*double((1<<16)-1));
    set_taget_pos_location(location);
}

// 设置多圈目标位置,单位为度
void HXC_DMCtrl::set_taget_pos_deg(float deg){
    set_taget_pos_rad(deg*PI/180.f);
}

// 设置最大力矩（0-1）
void HXC_DMCtrl::set_max_Torque(float _max_Torque){
    max_Torque = _max_Torque;
}

// 卸载使能
void HXC_DMCtrl::unload(){
    if (torque_func_handle != nullptr) {
        vTaskDelete(torque_func_handle);
        torque_func_handle = nullptr;
    }
}
// 使能电机
void HXC_DMCtrl::load(){
    if (torque_func_handle == nullptr) {
        xTaskCreate(torque_ctrl_task, "torque_ctrl_task", torque_task_stack_size, this, torque_task_Priority, &torque_func_handle);
    }
}
// 获取电机是否使能
bool HXC_DMCtrl::get_is_load(){
    return torque_func_handle != nullptr;
}

// 设置目标速度，单位：RPM，加速度控制
void HXC_DMCtrl::set_speed(float speed, float acce){
    taget_speed = speed;
    acceleration = acce;
    if (speed_func_handle == nullptr) {
        xTaskCreate(speed_contral_task, "speed_contral_task", speed_task_stack_size, this, speed_task_Priority, &speed_func_handle);
    }
}

// 获取目标位置
int64_t HXC_DMCtrl::get_location_target() {
    return location_taget;
}


// 获取转子目标速度
float HXC_DMCtrl::get_taget_speed(){
    return taget_speed;
}
// 设置电机加速度，单位：RPM/s
void HXC_DMCtrl::set_acceleration(float acce){
    acceleration = acce;
}

// 获取减速箱减速比
float HXC_DMCtrl::get_reduction_ratio(){
    return reduction_ratio;
}
// 设置速度环位置误差系数
void HXC_DMCtrl::set_speed_location_K(float _K){
    speed_location_K = _K;
}

// 获取闭环控制频率
int HXC_DMCtrl::get_dynamic_send_frequency(){
    return dynamic_send_frequency;
}

// 设置闭环控制频率 0~1000
void HXC_DMCtrl::set_control_frequency(uint16_t min, uint16_t max){
    if(min > max) {
        uint16_t temp = min;
        min = max;
        max = temp;
    }   
    if(min < 1) {
        min = 1;
    }
    if(max > 1000) {
        max = 1000;
    }
    control_frequency_max = max;
    control_frequency_min = min;
}

// 当电机的输出电流需要根据位置进行映射的时候，可以传入位置到电流映射函数的指针
// 例如电机做摇臂控制电机时，电流需要做三角函数映射，那么可以传入经过三角函数的位置到电流映射函数的指针
void HXC_DMCtrl::add_location_to_Torque_func(std::function<int(int64_t)> func){
    location_to_Torque_func = func;
}

// 力矩控制任务的入口函数
void HXC_DMCtrl::torque_ctrl_task(void* p){
    HXC_DMCtrl* motor = (HXC_DMCtrl*)p;
    auto xLastWakeTime = xTaskGetTickCount();
    uint16_t last_send_torque = 0;
    int delay_time = 0;//延时时间
    int last_culc_dynamic_frequency_time = 0;//上次计算动态频率的时间
    uint32_t last_send_num=motor->send_package_num;//上次发送的包的数量
    while (1) {
        if(motor->t_ff != last_send_torque){//如果力矩发生变化，立刻发送新的力矩
            motor->sendMITpakage();
            last_send_torque = motor->t_ff;
            motor->send_package_num++;
        }else{
            delay_time++;
            if(delay_time > (configTICK_RATE_HZ / motor->control_frequency_min)){//如果力矩没有变化，延时一段时间再发送新的力矩
                motor->sendMITpakage();
                last_send_torque = motor->t_ff;
                motor->send_package_num++;
                delay_time = 0;
            }
        }
        // 计算动态频率
        if(xTaskGetTickCount()- last_culc_dynamic_frequency_time > 10*configTICK_RATE_HZ / motor->control_frequency_min){
            motor->dynamic_send_frequency=float(motor->send_package_num - last_send_num)*1000.f/(xTaskGetTickCount()-last_culc_dynamic_frequency_time);
            last_send_num = motor->send_package_num;
            last_culc_dynamic_frequency_time = xTaskGetTickCount();
        }
        //控制频率
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / motor->control_frequency_max);
    }
}

// 速度控制任务的入口函数
void HXC_DMCtrl::speed_contral_task(void* n){
    HXC_DMCtrl* moto = (HXC_DMCtrl*) n;
    //上次更新时间
    int last_update_speed_time=now_time_us();
    moto->speed_pid_contraler.reset();
    //在unload过后出mu现扰动，再次load之后不会回到扰动前的位置
    moto->speed_location_taget = moto->get_location();
    //目标控制速度，和目标速度的区别是受加速度限制
    float taget_control_speed = moto->taget_speed;
    float last_taget_control_speed = moto->taget_speed;
    auto xLastWakeTime = xTaskGetTickCount ();

    uint16_t last_value = 0;
    while (1){
        
        float delta_time=1e-6*(now_time_us()-last_update_speed_time); 
        
        if(moto->acceleration==0){
            taget_control_speed = moto->taget_speed;
        }else{
            //如果启用了加速度控制
            if(std::abs(taget_control_speed-last_taget_control_speed)>delta_time*moto->acceleration){
                //根据加速度控制速度
                taget_control_speed = last_taget_control_speed+(taget_control_speed-last_taget_control_speed)>0?delta_time*moto->acceleration:-delta_time*moto->acceleration;
            }else{
                taget_control_speed = taget_control_speed;
            }
        }
        //更新上次的设定速度
        last_taget_control_speed = taget_control_speed;

        //由速度计算得到的目标位置
        moto->speed_location_taget+=moto->is_online()*65535.f
        *((taget_control_speed*2.f*PI/60.f)/(2.f*moto->Pmax))
        *delta_time;//位置误差,只有电机在线才计算累计位置
        //更新上次更新时间
        last_update_speed_time=now_time_us();

        //由速度误差和位置误差一同计算电流
        double err = 
        /*速度环的误差=*/(taget_control_speed - moto->get_speed_rpm()*moto->speed_reduction_ratio)
        +
        /*速度环位置误差比例系数=*/moto->speed_location_K/*这里的比例系数需要根据实际情况调整,比例系数speed_location_K可以理解为转子每相差一圈加 speed_location_K RPM速度补偿*/
        * 
        /*由速度计算得到的目标位置的误差*/(moto->speed_location_taget-moto->get_location())/65535.f;

        
        //计算控制力矩 [-1,1]
        float Torque = 
        /*位置映射的力矩=*/moto->location_to_Torque_func(moto->get_location())
        +
        /*PID控制器的计算力矩=*/moto->speed_pid_contraler.control(moto->is_online()*err);//电机在线才计算电流
        
        //限幅
        Torque=Torque<-1? -1:Torque>1?1:Torque;
        uint16_t Torque_int=(Torque*2047)+2047;//将力矩转换为0-4095的整数值
        moto->t_ff=Torque_int;
        //控制频率
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / moto->control_frequency_max);
    }
}
// 位置控制任务的入口函数
void HXC_DMCtrl::location_contral_task(void* n){
    HXC_DMCtrl* moto = (HXC_DMCtrl*) n;
    moto->location_pid_contraler.reset();//重置位置闭环控制器
    float speed=0;
    auto xLastWakeTime = xTaskGetTickCount();
    while (1){
        //位置闭环控制,由位置误差决定速度,再由速度误差决定力矩
        
        speed = moto->location_pid_contraler.control(moto->location_taget - moto->get_location());
        moto->taget_speed = speed;
        //控制频率
        xTaskDelayUntil(&xLastWakeTime, configTICK_RATE_HZ / moto->control_frequency_max);
    }
};
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