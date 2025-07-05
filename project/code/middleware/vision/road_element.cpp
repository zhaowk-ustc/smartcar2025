#include "road_element.h"
#include "vision_utils.h"
#include "point.h"

// 检测单侧边界异常函数
BoundaryAnomalyResult detect_boundary_anomaly(const vector<int16>& bounds)
{
    int consecutive_disappears = 0;  // 连续消失的数量
    int16 last_stable_bound = -1;    // 上一个稳定（非突变）的边界位置
    int position_mutations_count = 0; // 位置突变的累计次数

    // 从下往上扫描，检测连续的突变
    for (int h = bounds.size() - 1; h >= 0; h--)  // 从最后一行开始
    {
        int16 current_bound = bounds[h];

        // 检测消失突变（当前值为-1）
        if (current_bound == -1)
        {
            consecutive_disappears++;

            // 如果连续消失达到阈值，判定为消失突变
            if (consecutive_disappears >= CONSECUTIVE_DISAPPEAR_MUTATION_THRESHOLD)
            {
                return BoundaryAnomalyResult(BOUNDARY_DISAPPEAR, h, 1.0f);
            }
        }
        else  // 当前值有效
        {
            // 重置消失计数器（因为当前不是消失）
            consecutive_disappears = 0;

            // 检测位置突变（与上一个稳定点的距离）
            if (last_stable_bound != -1)
            {
                int change = abs(current_bound - last_stable_bound);
                if (change > BOUNDARY_MUTATION_THRESHOLD)
                {
                    position_mutations_count++;

                    // 如果位置突变次数达到阈值，判定为位置突变
                    if (position_mutations_count >= CONSECUTIVE_POSITION_MUTATION_THRESHOLD)
                    {
                        return BoundaryAnomalyResult(BOUNDARY_MUTATION, h, 1.0f);
                    }
                    // 注意：发生位置突变时，不更新last_stable_bound
                }
                else
                {
                    // 没有位置突变，这是一个稳定点
                    last_stable_bound = current_bound;
                    position_mutations_count = 0;  // 重置位置突变计数
                }
            }
            else
            {
                // 这是第一个有效点，设为稳定点
                last_stable_bound = current_bound;
                position_mutations_count = 0;
            }
        }
    }

    return BoundaryAnomalyResult();  // 返回无异常结果
}

// 内部环岛检测函数
RoadElementResult detect_roundabout_internal(const vector<int16>& left_bounds,
    const vector<int16>& right_bounds)
{
    int height = left_bounds.size();
    if (height < ROUNDABOUT_DETECTION_SAMPLES)
    {
        return RoadElementResult();  // 图像太小，无法检测环岛
    }

    // 选择检测区域：从底部开始的一定范围内
    int detection_start = height - ROUNDABOUT_DETECTION_SAMPLES;
    if (detection_start < 0) detection_start = 0;
    int detection_end = height - 1;

    // 计算左右边界的非线性度
    float left_nonlinearity = calculate_boundary_nonlinearity(left_bounds, detection_start, detection_end);
    float right_nonlinearity = calculate_boundary_nonlinearity(right_bounds, detection_start, detection_end);

    // 计算左右边界的平均宽度差距
    float total_width_diff = 0.0f;
    int valid_width_count = 0;

    for (int i = detection_start; i <= detection_end; i++)
    {
        if (left_bounds[i] != -1 && right_bounds[i] != -1)
        {
            int width = right_bounds[i] - left_bounds[i];
            total_width_diff += width;
            valid_width_count++;
        }
    }

    if (valid_width_count < ROUNDABOUT_DETECTION_SAMPLES / 2)
    {
        return RoadElementResult();  // 有效数据不足
    }

    float avg_width = total_width_diff / valid_width_count;

    // 环岛检测逻辑
    float confidence = 0.0f;

    // 判据1：环岛侧边界非线性度较大
    // 判据2：左右边界差距大于一定值

    if (left_nonlinearity > ROUNDABOUT_NONLINEARITY_THRESHOLD &&
        avg_width > ROUNDABOUT_WIDTH_DIFF_THRESHOLD)
    {
        // 左边界非线性度大，可能是左侧环岛
        // 进一步检查：左边界非线性度应该显著大于右边界
        if (left_nonlinearity > right_nonlinearity * 1.5f)
        {
            confidence = (left_nonlinearity - ROUNDABOUT_NONLINEARITY_THRESHOLD) / ROUNDABOUT_NONLINEARITY_THRESHOLD;
            if (confidence > 1.0f) confidence = 1.0f;
            return RoadElementResult(LEFT_ROUNDABOUT, detection_start, confidence);
        }
    }
    else if (right_nonlinearity > ROUNDABOUT_NONLINEARITY_THRESHOLD &&
        avg_width > ROUNDABOUT_WIDTH_DIFF_THRESHOLD)
    {
        // 右边界非线性度大，可能是右侧环岛
        // 进一步检查：右边界非线性度应该显著大于左边界
        if (right_nonlinearity > left_nonlinearity * 1.5f)
        {
            confidence = (right_nonlinearity - ROUNDABOUT_NONLINEARITY_THRESHOLD) / ROUNDABOUT_NONLINEARITY_THRESHOLD;
            if (confidence > 1.0f) confidence = 1.0f;
            return RoadElementResult(RIGHT_ROUNDABOUT, detection_start, confidence);
        }
    }

    return RoadElementResult();  // 未检测到环岛
}

// 统一的道路元素检测函数
RoadElementResult detect_road_element(const vector<int16>& left_bounds,
    const vector<int16>& right_bounds)
{
    // 检测左右边界的异常情况
    BoundaryAnomalyResult left_anomaly = detect_boundary_anomaly(left_bounds);
    BoundaryAnomalyResult right_anomaly = detect_boundary_anomaly(right_bounds);

    // 根据边界异常模式判断路口类型
    if (left_anomaly.type == BOUNDARY_MUTATION && right_anomaly.type == BOUNDARY_MUTATION)
    {
        // 两侧都是位置突变 → 十字路口，使用平均位置
        return RoadElementResult(CROSS_ROAD, (left_anomaly.position + right_anomaly.position) / 2);
    }
    else if (left_anomaly.type == BOUNDARY_MUTATION && right_anomaly.type == BOUNDARY_DISAPPEAR)
    {
        // 左边界位置突变，右边界消失 → 左直角，使用左边界位置
        return RoadElementResult(LEFT_TURN, left_anomaly.position);
    }
    else if (left_anomaly.type == BOUNDARY_DISAPPEAR && right_anomaly.type == BOUNDARY_MUTATION)
    {
        // 左边界消失，右边界位置突变 → 右直角，使用右边界位置
        return RoadElementResult(RIGHT_TURN, right_anomaly.position);
    }

    // 首先检测环岛
    RoadElementResult roundabout_result = detect_roundabout_internal(left_bounds, right_bounds);
    if (roundabout_result.type != NO_ELEMENT)
    {
        return roundabout_result;
    }


    // 其他情况均为无道路元素
    return RoadElementResult();
}

// 道路元素类型转换为字符串的辅助函数
const char* road_element_type_to_string(RoadElementType type)
{
    switch (type)
    {
        case NO_ELEMENT:                return "None";
        case LEFT_TURN:                 return "LTurn";
        case RIGHT_TURN:                return "RTurn";
        case CROSS_ROAD:                return "Cross";
        case LEFT_ROUNDABOUT:           return "LRound";
        case RIGHT_ROUNDABOUT:          return "RRound";
        default:                        return "Unknown";
    }
}
