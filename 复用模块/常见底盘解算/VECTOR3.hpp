/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 三维向量类
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-06 10:00:41
 */
#ifndef VECTOR3
#define VECTOR3
// 坐标系示意图,车正前方是Y正，右侧是X正，正上方是Z正
//          Z
//          |
//          |    Y
//          |   /
//    ______|__/
//   /     /| /
//  /_____/ |/
//  |     | O--------> X
//  |_____|/

#include <cmath> // 用于 sqrt
template <typename T>
struct vec3 {
    T x = 0;
    T y = 0;
    T z = 0;

    // 构造函数
    vec3(T x = 0, T y = 0, T z = 0) : x(x), y(y), z(z) {}

    // 向量加法
    vec3<T> operator+(const vec3<T>& other) const {
        return vec3<T>(x + other.x, y + other.y, z + other.z);
    }

    // 向量减法
    vec3<T> operator-(const vec3<T>& other) const {
        return vec3<T>(x - other.x, y - other.y, z - other.z);
    }

    // 标量乘法（支持 float/int/double）
    vec3<T> operator*(T scalar) const {
        return vec3<T>(x * scalar, y * scalar, z * scalar);
    }

    // 叉积（cross product）
    vec3<T> cross(const vec3<T>& other) const {
        return vec3<T>(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // 点积（dot product）
    T dot(const vec3<T>& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // 向量长度（模）
    T length() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // 单位向量（归一化）
    vec3<T> normalized() const {
        T len = length();
        if (len > 0) { // 避免除以零
            return vec3<T>(x / len, y / len, z / len);
        }
        return vec3<T>(); // 返回零向量
    }

    bool is_zero() const {
        return (x == 0 && y == 0 && z == 0);
    }
};
#endif