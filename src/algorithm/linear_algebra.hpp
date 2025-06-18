#pragma once

#include <cmath>
#include <optional>

#include <godot_cpp/core/math.hpp>
#include <godot_cpp/variant/rect2.hpp>
#include <godot_cpp/variant/vector2.hpp>

namespace godot {
namespace math {
struct Line {
    Vector2 p1 = {0, 0};
    Vector2 p2 = {0, 0};

    Line() = default;
    Line(const Vector2& p1, const Vector2& p2) : p1(p1), p2(p2) {}

    float get_length() const { return p1.distance_to(p2); }
    float get_slope() const { return (p2.y - p1.y) / (p2.x - p1.x); }            // y / x
    float get_angle() const { return atan2(p2.y - p1.y, p2.x - p1.x); }          // atan2(y, x)
    Vector2 get_normal() const { return Vector2(-get_slope(), 1).normalized(); } // 法线
    Vector2 get_direction() const { return (p2 - p1).normalized(); }             // 方向
    Vector2 get_projection(Vector2 point) const {
        Vector2 direction = get_direction();
        float distance    = direction.dot(point - p1);
        return p1 + direction * distance;
    }
    std::optional<Vector2> get_intersection(Line line) const { // 求交点
        float s1_x = p2.x - p1.x;
        float s1_y = p2.y - p1.y;
        float s2_x = line.p2.x - line.p1.x;
        float s2_y = line.p2.y - line.p1.y;

        float s, t; // intersection point
        s = (-s1_y * (p1.x - line.p1.x) + s1_x * (p1.y - line.p1.y)) / (-s2_x * s1_y + s1_x * s2_y);
        t = (s2_x * (p1.y - line.p1.y) - s2_y * (p1.x - line.p1.x)) / (-s2_x * s1_y + s1_x * s2_y);

        if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
            return p1 + (p2 - p1) * t;
        }
        return std::nullopt;
    }
    std::optional<Vector2> get_intersection_by_ray(Line line) const { // 求交点
        auto a = p2.y - p1.y;
        auto b = p1.x - p2.x;
        auto e = p1.x * p2.y - p1.y * p2.x;

        auto c = (line.p2.y - line.p1.y);
        auto d = (line.p1.x - line.p2.x);
        auto f = line.p1.x * line.p2.y - line.p1.y * line.p2.x;

        auto detDown = a * d - b * c;
        if (Math::is_zero_approx(detDown)) {
            return std::nullopt;
        }

        auto invDet = 1.0f / detDown;
        auto x      = (d * e - b * f) * invDet;
        auto y      = (a * f - c * e) * invDet;

        auto ab = p2 - p1;
        if (Math::is_zero_approx(ab.x)) {
            if ((y - p1.y) * (y - p2.y) > 0) {
                return std::nullopt;
            }
        } else if (Math::is_zero_approx(ab.y)) {
            if ((x - p1.x) * (x - p2.x) > 0) {
                return std::nullopt;
            }
        } else if ((x - p1.x) * (x - p2.x) > 0 || (y - p1.y) * (y - p2.y) > 0) {
            return std::nullopt;
        }

        auto p  = Vector2(x, y);
        auto op = p - line.p1;
        if (op.dot(line.get_direction()) < 0) {
            return std::nullopt;
        }
        return p;
    }
    bool is_on_line(Vector2 point) const { // 判断点是否在线上
        return Math::is_zero_approx((point - p1).dot(get_direction()));
    }
    Vector2 get_closest_point_on_segment(Vector2 point) const { // 求线段上最近点
        Vector2 projection = get_projection(point);
        if (is_on_line(projection)) {
            return projection;
        }
        return (point - p1).length() < (point - p2).length() ? p1 : p2;
    }
    Vector2 get_closest_point(Vector2 point) const { // 求最近点
        Vector2 projection = get_projection(point);
        if (is_on_line(projection)) {
            return projection;
        }
        return get_closest_point_on_segment(point);
    }
};

struct Circle {
    Vector2 center;
    real_t radius;

    Circle(Vector2 center, real_t radius) : center(center), radius(radius) {}
    bool is_on_circle(Vector2 point) const { // 判断点是否在圆上
        return Math::is_zero_approx((point - center).length() - radius);
    }
    Vector2 get_closest_point(Vector2 point) const { // 求最近点
        return center + (point - center).normalized() * radius;
    }
    Vector2 get_projection(Vector2 point) const { // 求投影
        return center + (point - center).normalized() * radius;
    }
    // 求圆到线段起点最近的交点
    std::optional<Vector2> get_intersection_point(Line other) const {
        Vector2 projection = other.get_projection(center);
        auto distance      = (projection - center).length();
        if (Math::is_equal_approx(distance, radius)) {
            return projection;
        }
        if (distance > radius) {
            return std::nullopt;
        }
        auto half_chord = Math::sqrt(radius * radius - distance * distance);
        auto direction  = (projection - other.p1).normalized();
        return projection + direction * half_chord; // 返回到起点最近的交点
    }
};
} // namespace math
} // namespace godot