/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 三，四舵轮运动学逆解算封装库
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-06 10:18:41
 */
#ifndef SteeringWheel_hpp
#define SteeringWheel_hpp
#include "VECTOR3.hpp" // 三维向量类
#include <functional> // std::function
#include <array> // std::array
//单个舵轮运动学参数
struct wheel_module{
    float speed; // 舵轮转速 单位：m/s
    float angle; // 舵轮转角 单位：弧度
};

/**
 * @brief :  三舵轮底盘运动学逆解算封装库
 * @return  {*}
 * @Author : qingmeijiupiao
 */
class SteeringWheel_3{
    public:
        /**
         * @brief :  构造函数，参数的坐标原点可以为任意位置，但需要是同一坐标系下
         * @param rotation_center :旋转中心位置坐标 单位：m
         * @param wheel1_position :舵轮1位置坐标 单位：m
         * @param wheel2_position :舵轮2位置坐标 单位：m
         * @param wheel3_position :舵轮3位置坐标 单位：m
         * @return  {*}
         * @Author : qingmeijiupiao
         */
        SteeringWheel_3(vec3<float> rotation_center,vec3<float> wheel1_position,vec3<float> wheel2_position,vec3<float> wheel3_position){
            this->wheel1_position = wheel1_position;
            this->wheel2_position = wheel2_position;
            this->wheel3_position = wheel3_position;
            this->rotation_center_vector = rotation_center;
        };

        /**
         * @brief : 设置零点回调函数,用于当输入速度为0时，特殊处理每个舵轮的速度和角度
         * @return  {*}
         * @Author : qingmeijiupiao
         * @param {function<std::array<wheel_module,3>(void)>} zero_point_callback  
         */
        void set_zero_point_callback(std::function<std::array<wheel_module,3>(SteeringWheel_3*)> zero_point_callback){
            this->zero_point_callback = zero_point_callback;
        } 

        /**
         * @brief : 设置舵轮速度和角度
         * @return  {std::array<wheel_module,3>} 三个舵轮的速度和角度
         * @Author : qingmeijiupiao
         * @param {vec3<float>} speed_vector 速度向量 xy单位：m/s z单位：rad/s
         */
        std::array<wheel_module,3> set_speed(vec3<float> speed_vector){
            //获取每个舵轮相对旋转中心的向量
            vec3<float> wheel1_vector = wheel1_position - rotation_center_vector; // 舵轮1相对旋转中心位置向量
            vec3<float> wheel2_vector = wheel2_position - rotation_center_vector; // 舵轮2相对旋转中心位置向量
            vec3<float> wheel3_vector = wheel3_position - rotation_center_vector; // 舵轮3相对旋转中心位置向量

            float wheel1_vector_length = wheel1_vector.length(); // 舵轮1相对旋转中心位置向量长度
            float wheel2_vector_length = wheel2_vector.length(); // 舵轮2相对旋转中心位置向量长度
            float wheel3_vector_length = wheel3_vector.length(); // 舵轮3相对旋转中心位置向量长度

            //计算每个舵轮垂直于旋转中心的矢量
            wheel1_vector = wheel1_vector.cross(vec3<float>(0,0,1));
            wheel2_vector = wheel2_vector.cross(vec3<float>(0,0,1));
            wheel3_vector = wheel3_vector.cross(vec3<float>(0,0,1));

            //如果速度矢量为0，调用零点回调函数
            if(speed_vector.is_zero()){
                if(zero_point_callback != nullptr){
                    return zero_point_callback(this); // 如果速度为0，调用零点回调函数

                }else{// 如果没有零点回调函数，返回上一次的舵轮角度
                    auto last_wheel_module_speed_angle = wheel_module_speed_angle; 
                    for(auto& w : last_wheel_module_speed_angle) w.speed = 0; // 将上一次的舵轮速度置为0
                    return last_wheel_module_speed_angle; // 返回上一次的舵轮角度
                }
            }


            //每个舵轮的速度矢量可以看作 平面速度矢量加上一个垂直于平面速度矢量的旋转速度矢量
            
            vec3<float> speed1(speed_vector.x,speed_vector.y,0); //平面速度矢量


            //单位化
            wheel1_vector = wheel1_vector.normalized();
            wheel2_vector = wheel2_vector.normalized();
            wheel3_vector = wheel3_vector.normalized();

            //计算每个舵轮的旋转速度矢量
            wheel1_vector=wheel1_vector*wheel1_vector_length;
            wheel2_vector=wheel2_vector*wheel2_vector_length;
            wheel3_vector=wheel3_vector*wheel3_vector_length;

            //加上平面速度矢量
            wheel1_vector = wheel1_vector + speed1;
            wheel2_vector = wheel2_vector + speed1;
            wheel3_vector = wheel3_vector + speed1;

            //计算每个舵轮的角度
            float wheel1_angle = atan2(wheel1_vector.y,wheel1_vector.x);
            float wheel2_angle = atan2(wheel2_vector.y,wheel2_vector.x);
            float wheel3_angle = atan2(wheel3_vector.y,wheel3_vector.x);

            //计算每个舵轮的速度
            float wheel1_speed = wheel1_vector.length();
            float wheel2_speed = wheel2_vector.length();
            float wheel3_speed = wheel3_vector.length();
            wheel_module_speed_angle = {wheel_module{wheel1_speed,wheel1_angle},wheel_module{wheel2_speed,wheel2_angle},wheel_module{wheel3_speed,wheel3_angle}};
            return wheel_module_speed_angle;
        }

        std::array<wheel_module,3> operator<<(vec3<float> speed_vector){
            return set_speed(speed_vector);
        };
        
        /**
         * @brief :  获取构造函数传入的舵轮位置坐标
         * @return  {*}
         * @Author : qingmeijiupiao
         * @param {int} wheel_number
         */
        vec3<float> get_wheel_position(int wheel_number){
            switch (wheel_number)
            {
                case 1:
                    return wheel1_position;
                case 2:
                    return wheel2_position;
                case 3:
                    return wheel3_position;
                default:
                    return vec3<float>(0,0,0);
            }
        };

        /**
         * @brief :  获取构造函数传入的旋转中心位置坐标
         * @return  {*}
         * @Author : qingmeijiupiao
         */
        vec3<float> get_rotation_center(){
            return rotation_center_vector;
        };

        /**
         * @brief :  修改旋转中心位置
         * @param  {*} rotation_center  旋转中心位置坐标,需要和构造函数传入的坐标系一致
         * @return  {*}
         * @Author : qingmeijiupiao
         */
        void change_rotation_center(vec3<float> rotation_center){
            this->rotation_center_vector = rotation_center;
        };

    protected:
        vec3<float> wheel1_position; // 舵轮1位置
        vec3<float> wheel2_position; // 舵轮2位置
        vec3<float> wheel3_position; // 舵轮3位置
        vec3<float> rotation_center_vector; // 旋转中心位置

        std::function<std::array<wheel_module,3>(SteeringWheel_3*)> zero_point_callback = nullptr; // 零点回调函数

        std::array<wheel_module,3> wheel_module_speed_angle = {wheel_module{0,0},wheel_module{0,0},wheel_module{0,0}}; // 上一次舵轮速度和角度
};


/**
 * @brief : 四舵轮底盘运动学逆解算封装库
 */
class SteeringWheel_4 {
public:
    /**
     * @brief 构造函数，参数的坐标原点可以为任意位置，但需要是同一坐标系下
     * @param rotation_center 旋转中心位置坐标 单位：m
     * @param wheel1_position 舵轮1位置坐标 单位：m
     * @param wheel2_position 舵轮2位置坐标 单位：m 
     * @param wheel3_position 舵轮3位置坐标 单位：m
     * @param wheel4_position 舵轮4位置坐标 单位：m
     */
    SteeringWheel_4(vec3<float> rotation_center, 
                    vec3<float> wheel1_position,
                    vec3<float> wheel2_position,
                    vec3<float> wheel3_position,
                    vec3<float> wheel4_position)
        : wheel1_position(wheel1_position),
            wheel2_position(wheel2_position),
            wheel3_position(wheel3_position),
            wheel4_position(wheel4_position),
            rotation_center_vector(rotation_center) {}

    /**
     * @brief 设置零点回调函数
     * @param zero_point_callback 回调函数，当输入速度为0时调用
     */
    void set_zero_point_callback(std::function<std::array<wheel_module,4>(SteeringWheel_4*)> zero_point_callback) {
        this->zero_point_callback = zero_point_callback;
    }

    /**
     * @brief 设置底盘速度
     * @param speed_vector 速度向量 xy单位：m/s z单位：rad/s
     * @return 四个舵轮的速度和角度
     */
    std::array<wheel_module,4> set_speed(vec3<float> speed_vector) {
        // 获取相对旋转中心的向量
        vec3<float> wheel1_vec = wheel1_position - rotation_center_vector;
        vec3<float> wheel2_vec = wheel2_position - rotation_center_vector;
        vec3<float> wheel3_vec = wheel3_position - rotation_center_vector;
        vec3<float> wheel4_vec = wheel4_position - rotation_center_vector;

        // 计算垂直向量（旋转分量方向）
        vec3<float> wheel1_rot = wheel1_vec.cross(vec3<float>(0,0,1)).normalized();
        vec3<float> wheel2_rot = wheel2_vec.cross(vec3<float>(0,0,1)).normalized();
        vec3<float> wheel3_rot = wheel3_vec.cross(vec3<float>(0,0,1)).normalized();
        vec3<float> wheel4_rot = wheel4_vec.cross(vec3<float>(0,0,1)).normalized();

        // 处理零速度情况
        if(speed_vector.is_zero()) {
            if(zero_point_callback) {
                return zero_point_callback(this);
            } else {
                auto last = wheel_module_speed_angle;
                for(auto& w : last) w.speed = 0;
                return last;
            }
        }

        // 计算旋转分量
        float omega = speed_vector.z;
        wheel1_rot = wheel1_rot * (omega * wheel1_vec.length());
        wheel2_rot = wheel2_rot * (omega * wheel2_vec.length());
        wheel3_rot = wheel3_rot * (omega * wheel3_vec.length());
        wheel4_rot = wheel4_rot * (omega * wheel4_vec.length());

        // 计算平面速度分量
        vec3<float> speed1(speed_vector.x, speed_vector.y, 0);
        vec3<float> speed2(speed_vector.x, speed_vector.y, 0);
        vec3<float> speed3(speed_vector.x, speed_vector.y, 0);
        vec3<float> speed4(speed_vector.x, speed_vector.y, 0);

        // 计算每个舵轮的速度矢量
        speed1 = speed1 + wheel1_rot;
        speed2 = speed2 + wheel2_rot;
        speed3 = speed3 + wheel3_rot;
        speed4 = speed4 + wheel4_rot;

        // 计算角度和速度
        wheel_module_speed_angle = {
            wheel_module{speed1.length(), atan2(speed1.y, speed1.x)},
            wheel_module{speed2.length(), atan2(speed2.y, speed2.x)},
            wheel_module{speed3.length(), atan2(speed3.y, speed3.x)},
            wheel_module{speed4.length(), atan2(speed4.y, speed4.x)}
        };

        return wheel_module_speed_angle;
    }

    std::array<wheel_module,4> operator<<(vec3<float> speed_vector) {
        return set_speed(speed_vector);
    }

    /**
     * @brief 获取舵轮位置
     * @param wheel_number 舵轮编号(1-4)
     * @return 舵轮位置坐标
     */
    vec3<float> get_wheel_position(int wheel_number) {
        switch(wheel_number) {
            case 1: return wheel1_position;
            case 2: return wheel2_position;
            case 3: return wheel3_position;
            case 4: return wheel4_position;
            default: return vec3<float>(0,0,0);
        }
    }

    /**
     * @brief 获取旋转中心位置
     */
    vec3<float> get_rotation_center() const {
        return rotation_center_vector;
    }

    /**
     * @brief 修改旋转中心位置
     * @param rotation_center 新的旋转中心坐标
     */
    void change_rotation_center(vec3<float> rotation_center) {
        rotation_center_vector = rotation_center;
    }

protected:
    vec3<float> wheel1_position;
    vec3<float> wheel2_position;
    vec3<float> wheel3_position;
    vec3<float> wheel4_position;
    vec3<float> rotation_center_vector;

    std::function<std::array<wheel_module,4>(SteeringWheel_4*)> zero_point_callback = nullptr;
    std::array< wheel_module,4 > wheel_module_speed_angle = {
        wheel_module{0,0}, wheel_module{0,0}, 
        wheel_module{0,0}, wheel_module{0,0}
    };
};

#endif