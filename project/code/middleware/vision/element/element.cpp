#include "track_path.h"
#include "element.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include <numeric>


// 多分支节点元素检测
// in_dir: 入口方向向量
// out_dirs: 所有出口方向向量
// 返回pair<元素类型, 下一个方向序号>
pair<ElementType, int> detect_branch_element(const Point2f& in_vec, const vector<Point2f>& out_vecs)
{
    ElementType type = ElementType::NORMAL;
    int out_idx = 0;
    if (out_vecs.size() <= 1)
    {
        return { type,out_idx };
    }
    vector<complex<float>> out_dirs;
    float abs_sin_sum = 0.0f;
    float sin_sum = 0.0f;

    complex<float> in_complex(in_vec.x(), in_vec.y());

    if ((abs(in_complex.real()) + abs(in_complex.imag())) < 0.5)
    {
        in_complex = { 0,-1 };
    }

    for (const auto& dir : out_vecs)
    {
        complex<float> out_complex(dir.x(), dir.y());

        out_complex /= in_complex;
        float cos = out_complex.real();
        float sin = out_complex.imag();
        out_dirs.push_back(out_complex);

        abs_sin_sum += abs(sin);
        sin_sum += sin;
    }
    switch (out_vecs.size())
    {
        case 3:
            if (abs(abs_sin_sum) <= 1.5)
            {
                // 用 sin_sum 的正负号区分左/右环岛
                if (sin_sum > 0)
                    type = ElementType::RIGHT_ROUNDABOUT;
                else
                    type = ElementType::LEFT_ROUNDABOUT;
                // 找到cos第二大的方向
                std::vector<std::pair<float, int>> cos_with_idx;
                for (int i = 0; i < out_dirs.size(); ++i)
                    cos_with_idx.emplace_back(out_dirs[i].real(), i);
                std::sort(cos_with_idx.begin(), cos_with_idx.end(),
                    [](const std::pair<float, int>& a, const std::pair<float, int>& b) { return a.first > b.first; });
                out_idx = cos_with_idx.size() > 1 ? cos_with_idx[1].second : 0;
            }
            else
            {
                type = ElementType::CROSS;
                // 找到cos最大的方向
                float max_cos = -2;
                int max_idx = 0;
                for (int i = 0; i < out_dirs.size(); ++i)
                {
                    float cos_val = out_dirs[i].real();
                    if (cos_val > max_cos)
                    {
                        max_cos = cos_val;
                        max_idx = i;
                    }
                }
                out_idx = max_idx;
            }
            break;
        case 2:
            type = ElementType::UNCERTAIN;
            {
                // 找到cos最大的方向
                float min_cos = 2;
                int min_idx = 0;
                for (int i = 0; i < out_dirs.size(); ++i)
                {
                    float cos_val = out_dirs[i].real();
                    if (cos_val > min_cos)
                    {
                        min_cos = cos_val;
                        min_idx = i;
                    }
                }
                out_idx = min_idx;
            }
            break;
        default:
            break;
    }
    return { type,out_idx };

}
