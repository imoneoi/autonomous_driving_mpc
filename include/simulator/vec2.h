#ifndef __CAR_MODEL_SIMULATOR_VEC2_H__
#define __CAR_MODEL_SIMULATOR_VEC2_H__

#include <cmath>
#include <algorithm>

template<typename T>
struct Vec2 {
    T x, y;

    inline Vec2<T> operator*(const T& rhs) const { return {x * rhs, y * rhs}; }
    inline Vec2<T> operator/(const T& rhs) const { return {x / rhs, y / rhs}; }

    inline Vec2<T> operator+(const Vec2<T>& rhs) const { return {x + rhs.x, y + rhs.y}; }
    inline Vec2<T> operator+=(const Vec2<T>& rhs) { (*this) = (*this) + rhs; return (*this); }

    inline T len() const { return std::sqrt(x * x + y * y); }
};

#endif