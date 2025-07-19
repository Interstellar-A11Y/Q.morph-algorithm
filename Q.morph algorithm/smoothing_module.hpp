#ifndef SMOOTHING_MODULE_HPP
#define SMOOTHING_MODULE_HPP

#include "mesh_structure.hpp"
#include <memory>
#include <vector>
#include <cmath>

namespace QMorph {

    // 局部平滑模块
    class SmoothingModule {
    private:
        std::shared_ptr<Mesh> mesh;
        double boundaryEdgeLengthRatioThreshold; // 边界边长度比阈值

    public:
        SmoothingModule(std::shared_ptr<Mesh> mesh)
            : mesh(mesh), boundaryEdgeLengthRatioThreshold(2.5) {}

        // 设置边界边长度比阈值
        void setBoundaryEdgeLengthRatioThreshold(double threshold) {
            boundaryEdgeLengthRatioThreshold = threshold;
        }

        // 平滑网格
        void smoothMesh() {
            // 1. 平滑内部节点
            smoothInteriorNodes();

            // 2. 平滑前沿节点
            smoothFrontierNodes();

            // 3. 检查并修正法向量倒置问题
            checkAndFixNormalInversion();
        }

        // 平滑内部节点
        void smoothInteriorNodes() {
            for (auto& node : mesh->nodes) {
                // 跳过边界节点
                if (node->isBoundary) continue;

                // 计算相邻节点的质心
                Vector3D centroid(0, 0, 0);
                int neighborCount = 0;

                // 查找相邻节点
                std::vector<std::shared_ptr<Node>> neighbors;
                for (auto& edge : mesh->edges) {
                    if (edge->node1 == node) {
                        neighbors.push_back(edge->node2);
                    }
                    else if (edge->node2 == node) {
                        neighbors.push_back(edge->node1);
                    }
                }

                // 计算质心
                for (auto& neighbor : neighbors) {
                    centroid = centroid + neighbor->coordinates;
                    ++neighborCount;
                }

                if (neighborCount > 0) {
                    centroid = centroid * (1.0 / neighborCount);

                    // 移动节点到质心位置
                    node->coordinates = centroid;
                }
            }
        }

        // 平滑前沿节点
        void smoothFrontierNodes() {
            // 遍历所有边
            for (auto& edge : mesh->edges) {
                if (!edge->isBoundary) continue;

                // 计算边的两个端点的相邻边长度比
                double lengthRatio1 = calculateBoundaryEdgeLengthRatio(edge->node1);
                double lengthRatio2 = calculateBoundaryEdgeLengthRatio(edge->node2);

                // 根据长度比选择平滑策略
                if (lengthRatio1 < boundaryEdgeLengthRatioThreshold) {
                    smoothFrontierNodeWithIsoParametric(edge->node1);
                }
                else {
                    smoothFrontierNodeWithAdvanced(edge->node1);
                }

                if (lengthRatio2 < boundaryEdgeLengthRatioThreshold) {
                    smoothFrontierNodeWithIsoParametric(edge->node2);
                }
                else {
                    smoothFrontierNodeWithAdvanced(edge->node2);
                }
            }
        }

        // 计算边界边长度比
        double calculateBoundaryEdgeLengthRatio(const std::shared_ptr<Node>& node) {
            // 查找所有相邻的边界边
            std::vector<std::shared_ptr<Edge>> boundaryEdges;
            for (auto& edge : mesh->edges) {
                if ((edge->node1 == node || edge->node2 == node) && edge->isBoundary) {
                    boundaryEdges.push_back(edge);
                }
            }

            // 计算长度比
            if (boundaryEdges.size() < 2) return 1.0;

            double minLength = boundaryEdges[0]->length();
            double maxLength = boundaryEdges[0]->length();

            for (size_t i = 1; i < boundaryEdges.size(); ++i) {
                double length = boundaryEdges[i]->length();
                minLength = std::min(minLength, length);
                maxLength = std::max(maxLength, length);
            }

            return maxLength / minLength;
        }

        // 使用等参方法平滑前沿节点
        void smoothFrontierNodeWithIsoParametric(const std::shared_ptr<Node>& node) {
            // 简化实现，实际需实现等参平滑+角度修正
            // 这里仅作示例，不进行实际平滑
        }

        // 使用高级方法平滑前沿节点
        void smoothFrontierNodeWithAdvanced(const std::shared_ptr<Node>& node) {
            // 简化实现，实际需基于相邻边长度和前沿状态计算目标位置
            // 这里仅作示例，不进行实际平滑
        }

        // 检查并修正法向量倒置问题
        void checkAndFixNormalInversion() {
            // 遍历所有节点
            for (auto& node : mesh->nodes) {
                Vector3D originalPosition = node->coordinates;

                // 计算节点移动后的新位置
                Vector3D newPosition = calculateSmoothedPosition(node);

                // 临时移动节点到新位置
                node->coordinates = newPosition;

                // 检查是否有法向量倒置的三角形
                bool hasInvertedTriangles = checkForInvertedTriangles(node);

                if (hasInvertedTriangles) {
                    // 逐步调整节点位置，直到没有倒置的三角形
                    Vector3D direction = newPosition - originalPosition;
                    double step = 0.1; // 步长

                    for (int i = 1; i <= 10; ++i) {
                        // 尝试更小的移动
                        Vector3D testPosition = originalPosition + direction * (step * i);
                        node->coordinates = testPosition;

                        if (!checkForInvertedTriangles(node)) {
                            // 找到合适的位置
                            break;
                        }

                        // 如果尝试了所有步长仍有倒置三角形，回到原始位置
                        if (i == 10) {
                            node->coordinates = originalPosition;
                        }
                    }
                }
            }
        }

        // 计算平滑后的节点位置
        Vector3D calculateSmoothedPosition(const std::shared_ptr<Node>& node) {
            // 简化实现，实际需根据平滑算法计算
            return node->coordinates;
        }

        // 检查节点周围是否有法向量倒置的三角形
        bool checkForInvertedTriangles(const std::shared_ptr<Node>& node) {
            // 遍历所有包含该节点的三角形
            for (auto& triangle : mesh->triangles) {
                if (triangle->isRemoved) continue;

                if (triangle->containsNode(node)) {
                    // 计算三角形的法向量
                    Vector3D v1 = triangle->nodes[1]->coordinates - triangle->nodes[0]->coordinates;
                    Vector3D v2 = triangle->nodes[2]->coordinates - triangle->nodes[0]->coordinates;
                    Vector3D normal = v1.cross(v2).normalize();

                    // 检查法向量是否与原始法向量一致
                    if (!triangle->hasConsistentNormal(normal)) {
                        return true;
                    }
                }
            }

            return false;
        }
    };

} // namespace QMorph

#endif // SMOOTHING_MODULE_HPP    