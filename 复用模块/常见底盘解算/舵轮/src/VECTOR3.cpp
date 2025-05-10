#ifndef VECTOR3_CPP
#define VECTOR3_CPP
/*
 * @version: no version
 * @LastEditors: qingmeijiupiao
 * @Description: 三维向量类实现
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-10 14:48:20
 */
#include "VECTOR3.hpp"
#include <cmath> // for std::sqrt
// 构造函数
template <typename T>
vec3<T>::vec3(T x, T y, T z) : x(x), y(y), z(z) {}

// 向量加法
template <typename T>
vec3<T> vec3<T>::operator+(const vec3<T>& other) const {
    return vec3<T>(x + other.x, y + other.y, z + other.z);
}

// 向量减法
template <typename T> 
vec3<T> vec3<T>::operator-(const vec3<T>& other) const {
    return vec3<T>(x - other.x, y - other.y, z - other.z);
}

// 标量乘法
template <typename T>
vec3<T> vec3<T>::operator*(T scalar) const {
    return vec3<T>(x * scalar, y * scalar, z * scalar);
}

// 叉积
template <typename T>
vec3<T> vec3<T>::cross(const vec3<T>& other) const {
    return vec3<T>(
        y * other.z - z * other.y,
        z * other.x - x * other.z, 
        x * other.y - y * other.x
    );
}

// 点积
template <typename T>
T vec3<T>::dot(const vec3<T>& other) const {
    return x * other.x + y * other.y + z * other.z;
}

// 向量长度
template <typename T>
T vec3<T>::length() const {
    return std::sqrt(x * x + y * y + z * z);
}

// 单位向量
template <typename T>
vec3<T> vec3<T>::normalized() const {
    T len = length();
    if (len > 0) { // 避免除以零
        return vec3<T>(x / len, y / len, z / len);
    }
    return vec3<T>(); // 返回零向量
}

// 判断是否为零向量
template <typename T>
bool vec3<T>::is_zero() const {
    return (x == 0 && y == 0 && z == 0);
}
#endif // VECTOR3_CPP