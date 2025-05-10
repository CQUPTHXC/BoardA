/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 通用舵轮运动学逆解算封装库实现
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-10 14:37:58
 */

#ifndef STEERING_WHEEL_CHASSIS_CPP
#define STEERING_WHEEL_CHASSIS_CPP
#include "SteeringWheelChassis.hpp"
template <size_t N>
template <typename... Args>
SteeringWheelChassis<N>::SteeringWheelChassis(vec3<float> rotation_center, Args&&... args)
    : rotation_center_vector(rotation_center) 
{
    // 检查参数数量
    static_assert(sizeof...(Args) == N, "Number of wheel positions must match template parameter N");
    // 检查参数类型
    static_assert((std::is_same_v<std::decay_t<Args>, vec3<float>> && ...), 
                "All arguments must be of type vec3<float>");
    // 初始化舵轮位置数组
    initWheelPositions(std::forward<Args>(args)...);
    // 初始化舵轮状态
    wheel_module_speed_angle.fill(WheelModule{0, 0});
}

// 设置零点回调函数
template <size_t N>
void SteeringWheelChassis<N>::set_zero_point_callback(
    std::function<std::array<WheelModule, N>(SteeringWheelChassis<N>*)> callback) 
{
    zero_point_callback = callback;
}

// 设置速度
template <size_t N>
std::array<WheelModule, N> SteeringWheelChassis<N>::set_speed(vec3<float> speed_vector) 
{
/* 
 * 数学原理：
 * 各舵轮速度 = 平移速度分量 + 旋转速度分量
 * 旋转分量由 (ω × r) 计算得出，其中：
 *   ω - 底盘旋转角速度矢量
 *   r - 舵轮到旋转中心的矢量
*/
    //处理速度为零的情况
    if (speed_vector.is_zero()) {
        if (zero_point_callback) {//调用回调函数
            return zero_point_callback(this);
        } else {//保持舵轮状态不变
            auto last = wheel_module_speed_angle;
            for (auto& w : last) w.speed = 0;
            return last;
        }
    }

    // 计算各舵轮速度和角度
    for (size_t i = 0; i < N; ++i) {
        // 计算舵轮相对于旋转中心的向量
        vec3<float> wheel_vec = wheel_positions[i] - rotation_center_vector;
        // 计算舵轮垂直旋转轴的单位向量
        vec3<float> wheel_rot = wheel_vec.cross(vec3<float>(0, 0, 1)).normalized();
        // 计算旋转速度分量
        wheel_rot = wheel_rot * (speed_vector.z * wheel_vec.length());
        // 计算平移速度分量
        vec3<float> speed(speed_vector.x, speed_vector.y, 0);
        // 合速度 = 平移速度 + 旋转速度
        vec3<float> total_speed = speed + wheel_rot;
        wheel_module_speed_angle[i] = {
            total_speed.length(),// 舵轮速度
            atan2(total_speed.y, total_speed.x)//舵轮角度
        };
    }

    return wheel_module_speed_angle;
}

// 操作符重载
template <size_t N>
std::array<WheelModule, N> SteeringWheelChassis<N>::operator<<(vec3<float> speed_vector) 
{
    return set_speed(speed_vector);
}

// 获取舵轮位置
template <size_t N>
vec3<float> SteeringWheelChassis<N>::get_wheel_position(size_t wheel_number) const 
{
    if (wheel_number > 0 && wheel_number <= N) {
        return wheel_positions[wheel_number - 1];
    }
    return vec3<float>(0, 0, 0);
}

// 获取旋转中心
template <size_t N>
vec3<float> SteeringWheelChassis<N>::get_rotation_center() const 
{
    return rotation_center_vector;
}

// 修改旋转中心
template <size_t N>
void SteeringWheelChassis<N>::change_rotation_center(vec3<float> rotation_center) 
{
    rotation_center_vector = rotation_center;
}

// 递归终止条件
template <size_t N>
void SteeringWheelChassis<N>::initWheelPositions() {}

// 递归展开参数包
template <size_t N>
template <typename T, typename... Args>
void SteeringWheelChassis<N>::initWheelPositions(T&& first, Args&&... rest) 
{
    static_assert(std::is_same_v<std::decay_t<T>, vec3<float>>, 
                 "All arguments must be of type vec3<float>");
    
    constexpr size_t index = N - sizeof...(rest) - 1;
    wheel_positions[index] = std::forward<T>(first);
    initWheelPositions(std::forward<Args>(rest)...);
}
#endif // STEERING_WHEEL_CHASSIS_CPP