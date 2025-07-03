// 使用示例：去除OpenCV依赖后的线跟踪API
#include "line_tracking.h"
#include <iostream>

void example_usage() {
    // 假设有一个二值图像数据
    int width = 320, height = 240;
    uint8_t* binary_image = new uint8_t[width * height];
    
    // 模拟一些二值图像数据
    memset(binary_image, 0, width * height);
    // ... 填充实际的二值图像数据 ...
    
    // 调用主处理函数
    LineTrackingGraph simplified_graph;
    uint8_t* visualization = extract_and_visualize_topology(
        binary_image, width, height, simplified_graph
    );
    
    if (visualization) {
        printf("成功提取拓扑结构，共 %zu 个节点\n", simplified_graph.nodes.size());
        printf("可视化图像数据已生成 (BGR格式，%dx%dx3)\n", width, height);
        
        // 打印前几个节点信息
        for (size_t i = 0; i < simplified_graph.nodes.size() && i < 5; ++i) {
            const auto& node = simplified_graph.nodes[i];
            printf("节点 %zu: 坐标(%d,%d), 类型=%d\n", 
                   i, node.pt.x, node.pt.y, node.type);
        }
        
        // 使用完毕后释放内存
        free(visualization);
    } else {
        printf("处理失败\n");
    }
    
    delete[] binary_image;
}

// 分步使用示例
void step_by_step_usage() {
    int width = 320, height = 240;
    uint8_t* binary_image = new uint8_t[width * height];
    
    // 1. 虚线修复
    std::vector<uint8_t> img_vec(binary_image, binary_image + width * height);
    repairDashedLinesAdvanced(img_vec, width, height, 15.0);
    
    // 2. BFS构建拓扑图
    LineTrackingGraph raw_graph = bfs_layer_midpoints(
        img_vec.data(), width, height, 5.0
    );
    
    // 3. 图简化
    LineTrackingGraph simplified = simplifyGraph(raw_graph, 2.0);
    
    printf("原始图：%zu 节点，简化后：%zu 节点\n", 
           raw_graph.nodes.size(), simplified.nodes.size());
    
    delete[] binary_image;
}
