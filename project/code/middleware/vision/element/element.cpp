#include "track_path.h"
#include "element.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <utility>
#include <complex>
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
    float cos_sum = 0.0f;
    float sin_sum = 0.0f;

    complex<float> in_complex(in_vec.x, in_vec.y);
    in_complex /= abs(in_complex);
    in_complex *= -1.0f; // 反向入射向量
    for (const auto& dir : out_vecs)
    {
        complex<float> out_complex(dir.x, dir.y);
        out_complex /= abs(out_complex);
        out_complex /= in_complex;
        float cos = out_complex.real();
        float sin = out_complex.imag();
        out_dirs.push_back(out_complex);

        cos_sum += cos;
        sin_sum += sin;
    }

    // 对 out_dirs 按辐角（arg）从小到大排序，并生成原序号的映射
    vector<int> out_dir_indices(out_dirs.size());
    iota(out_dir_indices.begin(), out_dir_indices.end(), 0);
    sort(out_dir_indices.begin(), out_dir_indices.end(), [&](int i, int j) {
        return arg(out_dirs[i]) < arg(out_dirs[j]);
        });
    // 排序后的out_dirs
    vector<complex<float>> sorted_out_dirs;
    for (int idx : out_dir_indices)
    {
        sorted_out_dirs.push_back(out_dirs[idx]);
    }
    // 如果后续需要用排序后的out_dirs，可以替换原out_dirs

    switch (out_vecs.size())
    {
        case 3:
            if (abs(sin_sum) >= 0.5)
            {
                type = ElementType::ROUNDABOUT;
            }
            else
            {
                type = ElementType::CROSS;
            }
            {
                float min_cos = std::numeric_limits<float>::max();
                int min_idx = 0;
                for (int i = 0; i < out_dirs.size(); ++i)
                {
                    float cos_val = out_dirs[i].real();
                    if (cos_val < min_cos)
                    {
                        min_cos = cos_val;
                        min_idx = i;
                    }
                }
                out_idx = out_dir_indices[min_idx];
            }
            break;
        case 2:
            type = ElementType::UNCERTAIN;
            break;
        default:
            break;
    }
    return { type,out_idx };

}
