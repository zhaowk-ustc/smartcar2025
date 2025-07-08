#include "element_detector.h"
#include <algorithm>
#include <cmath>
#include <complex>
#include <numeric>  // for std::iota
using namespace std;

static float cross_sin_sum(const std::vector<complex<float>>& vectors)
{
    if (vectors.size() < 2) return 0.0f;

    // 向量已归一化，按极角排序：先按虚部正负分块，然后按实部排序
    std::vector<size_t> order(vectors.size());
    std::iota(order.begin(), order.end(), 0);  // 初始化为 0, 1, 2, ...
    
    std::sort(order.begin(), order.end(), [&](size_t i, size_t j) {
        const auto& vi = vectors[i];
        const auto& vj = vectors[j];
        
        // 先按虚部正负分块
        bool i_positive = vi.imag() >= 0;
        bool j_positive = vj.imag() >= 0;
        
        if (i_positive != j_positive) {
            return i_positive > j_positive;  // 正虚部在前
        }
        
        // 同一块内按实部排序
        if (i_positive) {
            // 正虚部块：实部从大到小（从右到左）
            return vi.real() > vj.real();
        } else {
            // 负虚部块：实部从小到大（从左到右）
            return vi.real() < vj.real();
        }
    });
    
    const auto& angles = order;

    // 计算相邻向量夹角的sin值之和
    float sin_sum = 0.0f;
    for (size_t i = 0; i < angles.size(); ++i) {
        const auto& v1 = vectors[angles[i]];
        const auto& v2 = vectors[angles[(i + 1) % angles.size()]];
        float cross = v1.real() * v2.imag() - v1.imag() * v2.real();
        float len1 = std::abs(v1);
        float len2 = std::abs(v2);
        if (len1 > 0 && len2 > 0) {
            sin_sum += std::abs(cross) / (len1 * len2);
        }
    }
    return sin_sum;
}


/**
 * 基于角度与90度夹角的余弦值求和来区分十字路口和环岛
 */
static TrackElement detectNodeElement(const LineTrackingGraph& graph, int node_idx)
{
    TrackElement element;
    element.type = TrackElement::NONE;
    element.confidence = 0.0;
    const GraphNode& node = graph.getNode(node_idx);
    if (node.successors().size() < 2)
    {
        // 如果后继节点少于2个，直接返回空元素
        return element;
    }

    const auto& node = graph.getNode(node_idx);
    if (node.predecessor() == -2) return element; // 已删除的节点

    // 获取全部分支
    vector<uint16_t> branch_idxs;
    if (node.predecessor() >= 0)
    {
        branch_idxs.push_back(node.predecessor());
    }
    for (auto idx : node.successors())
    {

        branch_idxs.push_back(idx);
    }

    vector<complex<float>> branch_vectors;
    for (auto idx : branch_idxs)
    {
        Point pt = graph.getNode(idx).data();
        float branch_x = static_cast<float>(pt.x - node.data().x);
        float branch_y = static_cast<float>(pt.y - node.data().y);
        complex<float> branch_vector(branch_x, branch_y);
        branch_vector /= abs(branch_vector); // 归一化向量
        branch_vectors.push_back(branch_vector);
    }

    if (branch_vectors.size() >= 3)
    {
        // 直接基于向量计算分支间的正弦值求和
        float sin_sum = 0.0f;
        int pair_count = 0;

        if (branch_vectors.size() == 4)
        {
            // 4个分支：先按圆周顺序排序，然后计算相邻分支间的4个小夹角
            // 使用向量叉积进行圆周排序，避免反三角函数

            // 选择第一个向量作为参考
            std::vector<Point> sorted_vectors;
            sorted_vectors.push_back(branch_vectors[0]);

            // 找到剩余向量的圆周顺序
            std::vector<bool> used(branch_vectors.size(), false);
            used[0] = true;

            for (int step = 1; step < 4; ++step)
            {
                Point current = sorted_vectors.back();
                int next_idx = -1;
                double min_cross = 1e9; // 寻找最小的正叉积（逆时针最近）

                for (size_t i = 1; i < branch_vectors.size(); ++i)
                {
                    if (used[i]) continue;

                    const Point& candidate = branch_vectors[i];
                    // 计算叉积：current × candidate
                    double cross = current.x * candidate.y - current.y * candidate.x;

                    // 如果叉积为正且是目前最小的，或者所有剩余向量叉积都为负时选择最大的
                    bool should_select = false;
                    if (cross > 0)
                    {
                        if (min_cross > 1e8 || cross < min_cross)
                        {
                            should_select = true;
                        }
                    }
                    else if (min_cross > 1e8)
                    {
                        // 如果还没找到正叉积，在负叉积中选择最大的（最接近正方向）
                        if (next_idx == -1 || cross > min_cross)
                        {
                            should_select = true;
                        }
                    }

                    if (should_select)
                    {
                        next_idx = i;
                        min_cross = cross;
                    }
                }

                if (next_idx != -1)
                {
                    sorted_vectors.push_back(branch_vectors[next_idx]);
                    used[next_idx] = true;
                }
            }

            // 计算相邻分支间的4个小夹角的正弦值
            for (int i = 0; i < 4; ++i)
            {
                const Point& v1 = sorted_vectors[i];
                const Point& v2 = sorted_vectors[(i + 1) % 4];

                // 计算向量长度
                double len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
                double len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

                if (len1 > 0 && len2 > 0)
                {
                    // 使用叉积计算正弦值：sin(θ) = |v1 × v2| / (|v1| * |v2|)
                    double cross_product = abs(v1.x * v2.y - v1.y * v2.x);
                    double sin_value = cross_product / (len1 * len2);

                    sin_sum += sin_value;
                    pair_count++;
                }
            }
        }
        else
        {
            // 其他分支数：对所有分支对计算正弦值
            for (size_t i = 0; i < branch_vectors.size(); ++i)
            {
                for (size_t j = i + 1; j < branch_vectors.size(); ++j)
                {
                    const Point& v1 = branch_vectors[i];
                    const Point& v2 = branch_vectors[j];

                    // 计算向量长度
                    double len1 = sqrt(v1.x * v1.x + v1.y * v1.y);
                    double len2 = sqrt(v2.x * v2.x + v2.y * v2.y);

                    if (len1 > 0 && len2 > 0)
                    {
                        // 使用叉积计算正弦值：sin(θ) = |v1 × v2| / (|v1| * |v2|)
                        double cross_product = abs(v1.x * v2.y - v1.y * v2.x);
                        double sin_value = cross_product / (len1 * len2);

                        sin_sum += sin_value;
                        pair_count++;
                    }
                }
            }
        }

        // 归一化：除以分支对数量
        if (pair_count > 0)
        {
            sin_sum /= (double)pair_count;
        }

        printf("Junction analysis at (%d, %d): %d total branches (%d successors + predecessor), sin_sum=%.3f\n",
            node.data().x, node.data().y, (int)branch_vectors.size(), (int)valid_successors.size(), sin_sum);

        // 根据正弦值求和判断类型（针对不同分支数使用不同阈值）
        double crossroad_threshold;
        if (branch_vectors.size() == 3)
        {
            crossroad_threshold = 0.75; // 3分支的十字路口阈值
        }
        else if (branch_vectors.size() == 4)
        {
            crossroad_threshold = 0.85; // 4分支的十字路口阈值
        }
        else
        {
            crossroad_threshold = 0.80; // 其他分支数的默认阈值
        }

        printf("Using threshold %.2f for %d branches\n", crossroad_threshold, (int)branch_vectors.size());

        if (sin_sum >= crossroad_threshold)
        {
            // 十字路口：分支角度更接近90度，sin值更大
            element.type = TrackElement::CROSS;
            element.confidence = std::min(0.95, 0.5 + sin_sum * 0.4);

            if (branch_vectors.size() == 4)
            {
                if (sin_sum >= 0.9)
                {
                    element.description = "Standard perpendicular crossroad";
                }
                else
                {
                    element.description = "4-way crossroad";
                }
            }
            else if (branch_vectors.size() == 3)
            {
                element.description = "3-way junction (T-junction)";
            }
            else
            {
                element.description = "Multi-way crossroad";
            }
        }
        else
        {
            // 环岛：分支角度偏离90度较多，sin值较小
            element.type = TrackElement::ROUNDABOUT;
            element.confidence = std::min(0.95, 0.5 + (1.0 - sin_sum) * 0.4);

            if (branch_vectors.size() == 4)
            {
                element.description = "4-way roundabout junction";
            }
            else if (branch_vectors.size() == 3)
            {
                element.description = "3-way roundabout junction";
            }
            else
            {
                element.description = "Multi-way roundabout junction";
            }
        }

        printf("Detected: %s, confidence: %.2f (sin_sum: %.3f)\n",
            element.description.c_str(), element.confidence, sin_sum);
    }

    return element;
}

/**
 * 主要的赛道元素识别函数
 */
std::vector<TrackElement> analyzeTrackElements(const LineTrackingGraph& graph)
{
    std::vector<TrackElement> elements;

    printf("\n===== Track Element Analysis =====\n");
    printf("Analyzing topology with %d nodes\n", (int)graph.size());

    // 统一检测十字路口和环岛（基于角度与90度夹角的余弦值求和）
    for (size_t i = 0; i < graph.size(); ++i)
    {
        TrackElement junction = detectJunctionType(graph, (int)i);
        if (junction.confidence > 0.5)
        {
            elements.push_back(junction);
            printf("Detected: %s at (%d, %d), confidence: %.2f\n",
                junction.description.c_str(), junction.center.x, junction.center.y, junction.confidence);
        }
    }

    printf("Total elements detected: %d\n", (int)elements.size());
    return elements;
}

/**
 * 可视化赛道元素
 */
cv::Mat visualizeTrackElements(const cv::Mat& background, const LineTrackingGraph& graph,
    const std::vector<TrackElement>& elements)
{
    cv::Mat result;

    // 首先创建拓扑图可视化
    if (background.channels() == 1)
    {
        cv::cvtColor(background, result, cv::COLOR_GRAY2BGR);
    }
    else
    {
        result = background.clone();
    }

    // 绘制拓扑图的边
    for (size_t i = 0; i < graph.size(); ++i)
    {
        if (graph.getNode(i).predecessor() == -2) continue; // 跳过已删除的节点

        const Point& from = graph.getNode(i).data();
        auto successors = graph.getNode(i).successors();
        for (int successor_idx : successors)
        {
            if (successor_idx >= 0 && successor_idx < (int)graph.size() &&
                graph.getNode(successor_idx).predecessor() != -2)
            {
                const Point& to = graph.getNode(successor_idx).data();
                cv::line(result, cv::Point(from.x, from.y), cv::Point(to.x, to.y),
                    cv::Scalar(255, 255, 0), 2); // 黄色边
            }
        }
    }

    // 绘制拓扑图的节点
    for (size_t i = 0; i < graph.size(); ++i)
    {
        const auto& node = graph.getNode(i);
        if (node.predecessor() == -2) continue; // 跳过已删除的节点

        cv::Scalar color;
        int radius;

        // 根据后继数量判断节点类型
        auto successors = node.successors();
        int valid_successor_count = 0;
        for (int succ : successors)
        {
            if (succ >= 0 && succ < (int)graph.size() &&
                graph.getNode(succ).predecessor() != -2)
            {
                valid_successor_count++;
            }
        }

        if (valid_successor_count == 0)
        {
            // endpoint
            color = cv::Scalar(0, 0, 255); // 红色
            radius = 4;
        }
        else if (valid_successor_count > 1)
        {
            // branch
            color = cv::Scalar(255, 0, 255); // 品红色
            radius = 6;
        }
        else
        {
            // normal
            color = cv::Scalar(0, 255, 0); // 绿色
            radius = 3;
        }

        cv::circle(result, cv::Point(node.data().x, node.data().y), radius, color, -1);
    }

    // 在拓扑图上标注特殊元素
    for (const auto& element : elements)
    {
        cv::Point center(element.center.x, element.center.y);

        switch (element.type)
        {
            case TrackElement::CROSS:
                cv::circle(result, center, 15, cv::Scalar(0, 255, 255), 3); // 黄色圆圈
                cv::putText(result, "CROSS", cv::Point(center.x - 20, center.y - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2);
                break;
            case TrackElement::ROUNDABOUT:
                cv::circle(result, center, 25, cv::Scalar(255, 0, 255), 3); // 品红色圆圈
                cv::putText(result, "ROUNDABOUT", cv::Point(center.x - 40, center.y - 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 0, 255), 2);
                break;
            default:
                break;
        }
    }

    return result;
}
