#include "line_tracking.h"
#include "road_element.h"
#include "vision_utils.h"

// 寻找所有行的左边界，返回左边界数组
vector<int16> find_left_bounds(const uint8* image, uint16 width, uint16 height,
    const vector<int16>& extend_search_offsets)
{
    vector<int16> left_bounds(height, -1);  // 初始化为-1，表示未找到
    int16 previous_left_bound = -1;  // 存储前一个有效的左边界结果
    int search_center = width / 2;  // 初始搜索中心点

    // 如果传入的扩展搜索数组为空，使用默认值40填充
    vector<int16> effective_extend_offsets = extend_search_offsets;
    if (extend_search_offsets.empty())
    {
        effective_extend_offsets.assign(height, 10);
    }

    for (int h = height - 1; h >= 0; h--)
    {
        // 1. 确定有效的搜索中心点
        int effective_search_center = search_center;
        if (previous_left_bound != -1)
        {
            // 使用上一个有效的左边界作为搜索起点
            effective_search_center = previous_left_bound;
        }

        // 2. 寻找左边界
        int16 left_bound = -1;
        int white_point_found = -1;  // 找到的最近白点位置

        // 步骤1: 从搜索中心点往两边搜索最近的白点
        // 获取当前行的扩展搜索深度
        int16 current_extend_depth = (h < effective_extend_offsets.size()) ? effective_extend_offsets[h] : 40;
        int max_search_range = (width / 2 > current_extend_depth) ? width / 2 : current_extend_depth;

        for (int offset = 0; offset <= max_search_range; offset++)
        {
            // 先检查左边
            int left_x = effective_search_center - offset;
            if (left_x >= 0 && image[h * width + left_x] == 255)
            {
                white_point_found = left_x;
                break;
            }

            // 再检查右边（如果offset > 0）
            if (offset > 0)
            {
                int right_x = effective_search_center + offset;
                if (right_x < width && image[h * width + right_x] == 255)
                {
                    white_point_found = right_x;
                    break;
                }
            }

            // 如果超出扩展搜索范围且没找到白点，停止搜索
            if (offset >= current_extend_depth && white_point_found == -1)
            {
                break;
            }
        }

        // 步骤2: 如果找到白点，从该点向左搜索连通的最左白点
        if (white_point_found != -1)
        {
            // 从找到的白点开始向左搜索连通的最左边界
            for (int x = white_point_found; x >= 0; x--)
            {
                if (image[h * width + x] == 255)
                {
                    left_bound = x;
                }
                else
                {
                    // 遇到黑点时，使用扩展搜索防止像素缺陷
                    bool found_white_extend = false;
                    // 获取当前行的扩展搜索深度
                    int16 current_extend_depth = (h < effective_extend_offsets.size()) ? effective_extend_offsets[h] : 0;

                    for (int offset = 1; offset <= current_extend_depth; offset++)
                    {
                        int extended_x = x - offset;
                        if (extended_x >= 0 && image[h * width + extended_x] == 255)
                        {
                            // 找到白点，继续从这个位置搜索
                            x = extended_x;
                            found_white_extend = true;
                            break;
                        }
                    }
                    if (!found_white_extend)
                    {
                        break;  // 扩展搜索也没找到白点，停止搜索
                    }
                }
            }
        }

        left_bounds[h] = left_bound;

        // 判断是否应该将当前结果载入历史记录
        bool should_save_to_history = false;

        if (left_bound != -1)
        {
            // 如果有历史数据，检查是否发生突变
            bool is_sudden_change = false;
            if (previous_left_bound != -1)
            {
                // 检查与上一个有效边界的变化
                int change = abs(left_bound - previous_left_bound);
                // 如果变化超过阈值，认为是突变
                if (change > BOUNDARY_MUTATION_THRESHOLD)
                {
                    is_sudden_change = true;
                }
            }

            // 只有在不是突变的情况下才更新搜索中心点和保存到历史
            if (!is_sudden_change)
            {
                search_center = left_bound;
                should_save_to_history = true;
            }
        }

        // 只在非突变且找到有效边界时才保存到历史记录
        if (should_save_to_history)
        {
            previous_left_bound = left_bound;
        }
    }

    return left_bounds;
}

// 寻找所有行的右边界，返回右边界数组
vector<int16> find_right_bounds(const uint8* image, uint16 width, uint16 height,
    const vector<int16>& extend_search_offsets)
{
    vector<int16> right_bounds(height, -1);  // 初始化为-1，表示未找到
    int16 previous_right_bound = -1;  // 存储前一个有效的右边界结果
    int search_center = width / 2;  // 初始搜索中心点

    // 如果传入的扩展搜索数组为空，使用默认值40填充
    vector<int16> effective_extend_offsets = extend_search_offsets;
    if (extend_search_offsets.empty())
    {
        effective_extend_offsets.assign(height, 10);
    }

    for (int h = height - 1; h >= 0; h--)
    {
        // 1. 确定有效的搜索中心点
        int effective_search_center = search_center;
        if (previous_right_bound != -1)
        {
            // 使用上一个有效的右边界作为搜索起点
            effective_search_center = previous_right_bound;
        }

        // 2. 寻找右边界
        int16 right_bound = -1;
        int white_point_found = -1;  // 找到的最近白点位置

        // 步骤1: 从搜索中心点往两边搜索最近的白点
        // 获取当前行的扩展搜索深度
        int16 current_extend_depth = (h < effective_extend_offsets.size()) ? effective_extend_offsets[h] : 40;
        int max_search_range = (width / 2 > current_extend_depth) ? width / 2 : current_extend_depth;

        for (int offset = 0; offset <= max_search_range; offset++)
        {
            // 先检查右边
            int right_x = effective_search_center + offset;
            if (right_x < width && image[h * width + right_x] == 255)
            {
                white_point_found = right_x;
                break;
            }

            // 再检查左边（如果offset > 0）
            if (offset > 0)
            {
                int left_x = effective_search_center - offset;
                if (left_x >= 0 && image[h * width + left_x] == 255)
                {
                    white_point_found = left_x;
                    break;
                }
            }

            // 如果超出扩展搜索范围且没找到白点，停止搜索
            if (offset >= current_extend_depth && white_point_found == -1)
            {
                break;
            }
        }

        // 步骤2: 如果找到白点，从该点向右搜索连通的最右白点
        if (white_point_found != -1)
        {
            // 从找到的白点开始向右搜索连通的最右边界
            for (int x = white_point_found; x < width; x++)
            {
                if (image[h * width + x] == 255)
                {
                    right_bound = x;
                }
                else
                {
                    // 遇到黑点时，使用扩展搜索防止像素缺陷
                    bool found_white_extend = false;
                    // 获取当前行的扩展搜索深度
                    int16 current_extend_depth = (h < effective_extend_offsets.size()) ? effective_extend_offsets[h] : 0;

                    for (int offset = 1; offset <= current_extend_depth; offset++)
                    {
                        int extended_x = x + offset;
                        if (extended_x < width && image[h * width + extended_x] == 255)
                        {
                            // 找到白点，继续从这个位置搜索
                            x = extended_x;
                            found_white_extend = true;
                            break;
                        }
                    }
                    if (!found_white_extend)
                    {
                        break;  // 扩展搜索也没找到白点，停止搜索
                    }
                }
            }
        }

        right_bounds[h] = right_bound;

        // 判断是否应该将当前结果载入历史记录
        bool should_save_to_history = false;

        if (right_bound != -1)
        {
            // 如果有历史数据，检查是否发生突变
            bool is_sudden_change = false;
            if (previous_right_bound != -1)
            {
                // 检查与上一个有效边界的变化
                int change = abs(right_bound - previous_right_bound);
                // 如果变化超过阈值，认为是突变
                if (change > BOUNDARY_MUTATION_THRESHOLD)
                {
                    is_sudden_change = true;
                }
            }

            // 只有在不是突变的情况下才更新搜索中心点和保存到历史
            if (!is_sudden_change)
            {
                search_center = right_bound;
                should_save_to_history = true;
            }
        }

        // 只在非突变且找到有效边界时才保存到历史记录
        if (should_save_to_history)
        {
            previous_right_bound = right_bound;
        }
    }

    return right_bounds;
}

// 简化的道路中心线计算函数
// 参数：left - 左边界数组
//      right - 右边界数组
//      road_element - 道路元素检测结果（暂时保留接口）
// 返回：道路中心线，使用简单线性插值
vector<int16> process_road_line(const vector<int16>& left, const vector<int16>& right)
{
    int height = left.size();
    vector<int16> road_line(height, -1);

    // 1. 基础道路中心线计算
    for (int i = 0; i < height; i++)
    {
        if (left[i] != -1 && right[i] != -1)
        {
            // 两边界都有效，计算中心线
            road_line[i] = (left[i] + right[i]) / 2;
        }
        else if (left[i] != -1 && right[i] == -1)
        {
            // 只有左边界有效，估算道路宽度为40像素
            road_line[i] = left[i];
        }
        else if (left[i] == -1 && right[i] != -1)
        {
            // 只有右边界有效，估算道路宽度为40像素
            road_line[i] = right[i];
        }
        else
        {
            // 两边界都无效，保持-1，表示道路结束
            road_line[i] = 40;  // 保持-1表示该行没有有效的道路信息
        }
        // 两边界都无效时保持-1，表示道路结束
    }

    return road_line;
}

// 偏差计算函数，结合道路元素检测结果
float get_bias(const vector<int16>& road_line,
    const RoadElementResult& road_element, uint16 width)
{
    int height = road_line.size();
    if (height < 10) return 0.0f;

    float center_x = width / 2.0f;  // 图像中心点
    float bias = 0.0f;

    // 1. 基础位置偏差计算
    float position_bias = 0.0f;
    float total_weight = 0.0f;

    // 分区域计算偏差（近、中、远）
    int near_start = static_cast<int>(height * 0.7f);   // 近区域：70%-100%
    int mid_start = static_cast<int>(height * 0.4f);    // 中区域：40%-70%
    int far_start = static_cast<int>(height * 0.1f);    // 远区域：10%-40%

    // 近区域权重最高
    for (int i = near_start; i < height; i++)
    {
        if (road_line[i] != -1)
        {
            float local_bias = (road_line[i] - center_x) / center_x;
            float weight = 3.0f;
            position_bias += local_bias * weight;
            total_weight += weight;
        }
    }

    // 中区域权重中等
    for (int i = mid_start; i < near_start; i++)
    {
        if (road_line[i] != -1)
        {
            float local_bias = (road_line[i] - center_x) / center_x;
            float weight = 2.0f;
            position_bias += local_bias * weight;
            total_weight += weight;
        }
    }

    // 远区域权重最低
    for (int i = far_start; i < mid_start; i++)
    {
        if (road_line[i] != -1)
        {
            float local_bias = (road_line[i] - center_x) / center_x;
            float weight = 1.0f;
            position_bias += local_bias * weight;
            total_weight += weight;
        }
    }

    if (total_weight > 0.01f)
    {
        position_bias /= total_weight;
    }

    // 2. 道路线曲率分析
    float curvature_bias = 0.0f;
    // vector<pair<float, float>> valid_points;

    // // 收集有效点用于曲率计算
    // int curvature_start = height * 0.3f;
    // int curvature_end = height * 0.9f;
    // for (int i = curvature_start; i < curvature_end; i++)
    // {
    //     if (road_line[i] != -1)
    //     {
    //         valid_points.push_back({ (float)i, (float)road_line[i] });
    //     }
    // }

    // if (valid_points.size() >= 5)
    // {
    //     curvature_bias = calculate_quadratic_curvature(valid_points);
    // }

    // 3. 道路元素自适应策略
    float element_bias = 0.0f;
    // float element_weight = 1.0f;

    // if (road_element.type != NO_ELEMENT)
    // {
    //     // 计算元素位置相对于当前位置的偏差
    //     float element_position_ratio = (float)road_element.position / height;
    //     int element_position = road_element.position;

    //     switch (road_element.type)
    //     {
    //         case LEFT_TURN:
    //             // 左转：增加右偏趋势，提前准备左转
    //             element_bias = 0.2f * (1.0f - element_position_ratio);
    //             element_weight = 1.3f;
    //             break;

    //         case RIGHT_TURN:
    //             // 右转：增加左偏趋势，提前准备右转
    //             element_bias = -0.2f * (1.0f - element_position_ratio);
    //             element_weight = 1.3f;
    //             break;

    //         case CROSS_ROAD:
    //             // 十字路口：保持中央，减少偏差
    //             element_bias = -position_bias * 0.3f;
    //             element_weight = 1.2f;
    //             break;

    //         case LEFT_ROUNDABOUT:
    //             // 左环岛：增加右偏，准备进入环岛
    //             element_bias = 0.3f * (1.0f - element_position_ratio);
    //             element_weight = 1.5f;
    //             break;

    //         case RIGHT_ROUNDABOUT:
    //             // 右环岛：增加左偏，准备进入环岛
    //             element_bias = -0.3f * (1.0f - element_position_ratio);
    //             element_weight = 1.5f;
    //             break;

    //         default:
    //             element_bias = 0.0f;
    //             element_weight = 1.0f;
    //             break;
    //     }
    // }

    // 4. 综合计算最终偏差
    bias = position_bias + curvature_bias + element_bias;

    // 5. 偏差限制和滤波
    if (bias > 1.0f) bias = 1.0f;
    if (bias < -1.0f) bias = -1.0f;

    return bias;
}