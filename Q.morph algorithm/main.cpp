#include "mesh_structure.hpp"
#include <iostream>
#include <vector>
#include "main.hpp"

int main() {
    try {
        // 创建Q-Morph控制器
        QMorph::QMorphController controller;

        // 设置尺寸函数（这里使用均匀尺寸）
        auto sizeFunction = std::make_shared<QMorph::UniformSizeFunction>(0.1);
        controller.setSizeFunction(sizeFunction);

        // 定义边界点（这里使用一个简单的矩形边界作为示例）
        std::vector<QMorph::Vector3D> boundaryPoints = {
            QMorph::Vector3D(0, 0, 0),
            QMorph::Vector3D(1, 0, 0),
            QMorph::Vector3D(1, 1, 0),
            QMorph::Vector3D(0, 1, 0)
        };

        // 定义边界环（一个闭合的边界）
        std::vector<std::vector<int>> boundaryLoops = {
            {0, 1, 2, 3} // 矩形的四个顶点索引
        };

        // 执行Q-Morph算法
        controller.execute(boundaryPoints, boundaryLoops, "output.vtk");

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }
}