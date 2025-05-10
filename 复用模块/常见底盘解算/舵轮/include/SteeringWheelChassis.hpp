/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 通用舵轮运动学逆解算封装库声明（支持2轮及以上）
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-10 14:51:37
 */
#ifndef STEERING_WHEEL_CHASSIS_HPP
#define STEERING_WHEEL_CHASSIS_HPP

#include "VECTOR3.hpp"
#include <functional>
#include <array>
#include <utility>

// 单个舵轮运动学参数
struct WheelModule {
    float speed; // 舵轮转速 单位：m/s
    float angle; // 舵轮转角 单位：弧度 朝向X轴(底盘右侧)为0
};
/**
 * @class SteeringWheelChassis
 * @tparam N 舵轮数量，必须≥2
 * @brief 通用舵轮底盘运动学控制器
 * @note 坐标系约定:
 *       车正前方是Y正，右侧是X正，正上方是Z正
 *          Z
 *          |
 *          |    Y
 *          |   /
 *    ______|  /
 *   /     /| /
 *  /_____/ |/
 *  |     | O--------> X
 *  |_____|/
 */
 */
template <size_t N>
class SteeringWheelChassis {
public:
    /**
     * @brief 构造函数（可变参数模板版本）
     * 
     * @param rotation_center 底盘旋转中心坐标，单位：米(m)
     * @param args N个vec3<float>类型的舵轮位置参数，单位：米(m)
     * 
     * @note 参数约束：
     * 1. 参数数量必须精确匹配模板参数N
     * 2. 所有位置参数必须是vec3<float>类型
     * 3. z坐标将被忽略，建议设为0
     * 
     * @example 
     * // 三舵轮系统初始化
     * SteeringWheelChassis<3> chassis(
     *     vec3<float>{0,0,0},  // 旋转中心
     *     vec3<float>{1,0,0},  // 轮1
     *     vec3<float>{-0.5,0.866,0}, // 轮2
     *     vec3<float>{-0.5,-0.866,0} // 轮3
     * );
     */
    template <typename... Args>
    SteeringWheelChassis(vec3<float> rotation_center, Args&&... args);

    /**
     * @brief 设置零点速度回调函数
     * 
     * @param callback 回调函数，当输入速度为0时调用
     * 
     * @details 当底盘速度矢量为0时，系统将：
     * 1. 如果有设置回调，调用回调函数获取舵轮状态
     * 2. 否则保持各舵轮角度不变，速度设为0
     * 
     * 回调函数应返回包含N个WheelModule的数组，指定各舵轮在零速时的状态。
     * 典型应用场景：
     * - 保持舵轮最后有效角度
     * - 执行特殊归位动作
     */
    void set_zero_point_callback(
        std::function<std::array<WheelModule, N>(SteeringWheelChassis<N>*)> callback
    );
    
    /**
     * @brief 设置底盘运动速度
     * 
     * @param speed_vector 速度矢量(x,y,ω)：
     *   - x: 横向速度(m/s)
     *   - y: 前进速度(m/s)
     *   - z: 旋转角速度(rad/s)
     * @return std::array<WheelModule, N> 各舵轮的状态
     * 
     * @note 坐标系约定：
     * - x轴正向为底盘右侧
     * - y轴正向为底盘前方
     * - ω逆时针方向为正
     */
    std::array<WheelModule, N> set_speed(vec3<float> speed_vector);
    
    /**
     * @brief 速度设置操作符重载
     * @see set_speed()
     */
    std::array<WheelModule, N> operator<<(vec3<float> speed_vector);
    
    /**
     * @brief 获取指定舵轮的位置坐标
     * 
     * @param wheel_number 舵轮编号(1~N)
     * @return vec3<float> 舵轮位置坐标
     * @throws 无异常，但输入无效编号时返回(0,0,0)
     */
    vec3<float> get_wheel_position(size_t wheel_number) const;
    
    /**
     * @brief 获取当前旋转中心坐标
     */
    vec3<float> get_rotation_center() const;
    
    /**
     * @brief 修改旋转中心坐标
     * 
     * @param rotation_center 新的旋转中心坐标
     * 
     * @note 修改后立即生效，会影响后续所有速度计算
     */
    void change_rotation_center(vec3<float> rotation_center);

protected:
    std::array<vec3<float>, N> wheel_positions; ///< 各舵轮位置坐标数组
    vec3<float> rotation_center_vector;         ///< 当前旋转中心坐标
    
    /// 零点速度回调函数（默认为nullptr）
    std::function<std::array<WheelModule, N>(SteeringWheelChassis<N>*)> zero_point_callback = nullptr;
    
    /// 各舵轮当前状态（速度+角度）
    std::array<WheelModule, N> wheel_module_speed_angle;

private:
    /// 递归终止条件（空参数包处理）
    void initWheelPositions();
    
    /**
     * @brief 递归初始化舵轮位置数组
     * 
     * @tparam T 当前参数类型（自动推导）
     * @tparam Args 剩余参数类型包
     * @param first 当前处理的参数
     * @param rest 剩余参数包
     * 
     * @note 实现细节：
     * 1. 使用递归展开参数包
     * 2. 通过索引计算确定存储位置
     * 3. 使用完美转发保持参数值类别
     */
    template <typename T, typename... Args>
    void initWheelPositions(T&& first, Args&&... rest);
};

// 为常用舵轮数量定义别名
// 3轮舵轮底盘
using SteeringWheelChassis3 = SteeringWheelChassis<3>;
// 4轮舵轮底盘
using SteeringWheelChassis4 = SteeringWheelChassis<4>;

#endif // STEERING_WHEEL_CHASSIS_HPP