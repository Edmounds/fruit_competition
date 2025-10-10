#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <cmath> 

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @class Utils
 * @brief 一个包含通用静态辅助函数的工具类
 */
class Utils {
public:
    /**
     * @brief 默认构造函数 (通常工具类不需要实例化)
     */
    Utils() = default;

    /**
     * @brief 默认析构函数
     */
    ~Utils() = default;

    /**
     * @brief (静态方法) 计算从当前位置向量到目标位置向量需要转动的角度
     * @details 两个向量的起点均为坐标系原点(0,0)。
     * @param current_position 当前位置的2D坐标, 格式为 {x, y}
     * @param target_position 目标位置的2D坐标, 格式为 {x, y}
     * @return double 返回从当前向量旋转到目标向量的最短角度 (弧度制)。
     * 正值表示逆时针, 负值表示顺时针。范围在 [-PI, PI] 内。
     * @throw std::invalid_argument 如果输入的向量维度不为2。
     */
    static double CalculateRelativeAngle(const std::vector<double>& current_position, const std::vector<double>& target_position);

    // 你未来可以向这个类中添加更多的静态工具函数
    // static double AnotherUtilityFunction(...);
};

#endif // UTILS_H