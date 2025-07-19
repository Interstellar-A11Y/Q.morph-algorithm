#ifndef GLOBAL_SMOOTHING_OUTPUT_MODULE_HPP
#define GLOBAL_SMOOTHING_OUTPUT_MODULE_HPP

#include "mesh_structure.hpp"
#include <memory>
#include <vector>
#include <string>

namespace QMorph {

    // 全局平滑与结果输出模块
    class GlobalSmoothingOutputModule {
    private:
        std::shared_ptr<Mesh> mesh;
        int globalSmoothingIterations; // 全局平滑迭代次数

    public:
        GlobalSmoothingOutputModule(std::shared_ptr<Mesh> mesh)
            : mesh(mesh), globalSmoothingIterations(3) {}

        // 设置全局平滑迭代次数
        void setGlobalSmoothingIterations(int iterations) {
            globalSmoothingIterations = iterations;
        }

        // 执行全局平滑
        void performGlobalSmoothing() {
            for (int i = 0; i < globalSmoothingIterations; ++i) {
                smoothMeshOnce();
            }
        }

        // 执行一次全局平滑
        void smoothMeshOnce() {
            // 复制当前节点位置
            std::vector<Vector3D> oldPositions(mesh->nodes.size());
            for (size_t i = 0; i < mesh->nodes.size(); ++i) {
                oldPositions[i] = mesh->nodes[i]->coordinates;
            }

            // 遍历所有节点，计算新位置
            for (auto& node : mesh->nodes) {
                // 跳过边界节点
                if (node->isBoundary) continue;

                // 计算新位置
                Vector3D newPosition = calculateConstrainedLaplacianPosition(node);

                // 只有当新位置能改善形状质量时才移动
                if (wouldPositionImproveQuality(node, newPosition)) {
                    node->coordinates = newPosition;
                }
            }
        }

        // 计算约束拉普拉斯位置
        Vector3D calculateConstrainedLaplacianPosition(const std::shared_ptr<Node>& node) {
            // 简化实现，实际需计算相邻节点的加权平均位置
            return node->coordinates;
        }

        // 判断新位置是否能改善形状质量
        bool wouldPositionImproveQuality(const std::shared_ptr<Node>& node,
            const Vector3D& newPosition) {
            // 简化实现，实际需比较新旧位置的质量指标
            return true;
        }

        // 输出网格结果
        void outputMesh(const std::string& filename, const std::string& format = "vtk") {
            if (format == "vtk") {
                mesh->exportToVTK(filename);
            }
            else if (format == "stl") {
                // 实现STL输出
            }
            else if (format == "inp") {
                // 实现INP输出
            }
            else {
                throw std::runtime_error("不支持的输出格式: " + format);
            }
        }

        // 输出网格质量报告
        void outputMeshQualityReport(const std::string& filename) {
            std::ofstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("无法打开文件: " + filename);
            }

            auto quality = mesh->calculateMeshQuality();

            file << "网格质量报告\n";
            file << "------------------------\n";
            file << "四边形数量: " << mesh->quads.size() << "\n";
            file << "不规则节点数量: " << quality.irregularNodeCount << "\n";
            file << "平均质量指标(β): " << quality.avgQuality << "\n";
            file << "最小质量指标(β): " << quality.minQuality << "\n";
            file << "最大质量指标(β): " << quality.maxQuality << "\n";

            file.close();
        }
    };

} // namespace QMorph

#endif // GLOBAL_SMOOTHING_OUTPUT_MODULE_HPP    