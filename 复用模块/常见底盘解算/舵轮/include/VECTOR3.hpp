/*
 * @version: no version 
 * @LastEditors: qingmeijiupiao
 * @Description: 三维向量类声明
 * @author: qingmeijiupiao
 * @LastEditTime: 2025-05-10 14:51:45
 */
#ifndef VECTOR3_HPP
#define VECTOR3_HPP

/**
 * @brief 三维向量模板类
 * @tparam T 向量元素类型 (float/double/int等)
 * 
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
template <typename T>
struct vec3 {
    T x = 0;
    T y = 0;
    T z = 0;

    // 构造函数
    vec3(T x = 0, T y = 0, T z = 0);

    // 向量加法
    vec3<T> operator+(const vec3<T>& other) const;
    
    // 向量减法 
    vec3<T> operator-(const vec3<T>& other) const;
    
    // 标量乘法
    vec3<T> operator*(T scalar) const;
    
    // 叉积
    vec3<T> cross(const vec3<T>& other) const;
    
    // 点积
    T dot(const vec3<T>& other) const;
    
    // 向量长度
    T length() const;
    
    // 单位向量
    vec3<T> normalized() const;
    
    // 判断是否为零向量
    bool is_zero() const;
};

// 包含实现文件
#include "VECTOR3.ipp"

#endif // VECTOR3_HPP