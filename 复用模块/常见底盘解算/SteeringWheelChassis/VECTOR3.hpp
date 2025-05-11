// VECTOR3.h
#ifndef VECTOR3_HPP
#define VECTOR3_HPP

#include <cmath> // for std::sqrt

template <typename T>
class vec3 {
public:
    T x, y, z;
    
    // 构造函数
    vec3(T _x = 0, T _y = 0, T _z = 0) : x(_x), y(_y), z(_z) {}
    
    // 向量加法
    vec3<T> operator+(const vec3<T>& other) const {
        return vec3<T>(x + other.x, y + other.y, z + other.z);
    }
    
    // 向量减法
    vec3<T> operator-(const vec3<T>& other) const {
        return vec3<T>(x - other.x, y - other.y, z - other.z);
    }
    
    // 标量乘法
    vec3<T> operator*(T scalar) const {
        return vec3<T>(x * scalar, y * scalar, z * scalar);
    }
    
    // 叉积
    vec3<T> cross(const vec3<T>& other) const {
        return vec3<T>(
            y * other.z - z * other.y,
            z * other.x - x * other.z, 
            x * other.y - y * other.x
        );
    }
    
    // 点积
    T dot(const vec3<T>& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    
    // 向量长度
    T length() const {
        return std::sqrt(x * x + y * y + z * z);
    }
    
    // 单位向量
    vec3<T> normalized() const {
        T len = length();
        if (len > 0) {
            return vec3<T>(x / len, y / len, z / len);
        }
        return vec3<T>();
    }
    
    // 判断是否为零向量
    bool is_zero() const {
        return (x == 0 && y == 0 && z == 0);
    }
};

#endif // VECTOR3_HPP