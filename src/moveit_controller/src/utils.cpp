#include "moveit_controller/utils.h"
#include <stdexcept> // 用于 std::invalid_argument

// 使用 using 声明
using std::vector;
using std::atan2;
using std::invalid_argument;

// 静态成员函数的实现
double Utils::CalculateRelativeAngle(const vector<double>& current_position, const vector<double>& target_position) {
    // 参数校验
    if (current_position.size() != 2 || target_position.size() != 2) {
        // 抛出异常
        throw invalid_argument("Error: Position vectors must have 2 elements (x, y).");
    }

    // 使用 atan2(y, x) 计算每个向量与X轴正方向的夹角
    double angle_current = atan2(current_position[1], current_position[0]);
    double angle_target = atan2(target_position[1], target_position[0]);

    // 角度差即为需要转动的角度
    double delta_angle = angle_target - angle_current;
    
    // // 将角度规范化到 [-PI, PI] 范围内, 确保返回的是最短的旋转角度
    // while (delta_angle > M_PI) {
    //     delta_angle -= 2 * M_PI;
    // }
    // while (delta_angle <= -M_PI) {
    //     delta_angle += 2 * M_PI;
    // }
    
    return delta_angle;
}

// 未来其他工具函数的实现可以放在这里
// double Utils::AnotherUtilityFunction(...) {
//     ...
// }