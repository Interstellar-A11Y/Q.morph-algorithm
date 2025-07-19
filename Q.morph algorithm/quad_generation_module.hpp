#ifndef QUAD_GENERATION_MODULE_HPP
#define QUAD_GENERATION_MODULE_HPP

#include "mesh_structure.hpp"
#include "front_manager_module.hpp"
#include <memory>
#include <vector>
#include <set>
#include <cmath>
#include <stdexcept>

namespace QMorph {

    // 四边形生成模块
    class QuadGenerationModule {
    private:
        std::shared_ptr<Mesh> mesh;
        std::shared_ptr<FrontManagerModule> frontManager;
        double sideSelectionAngleThreshold; // 侧边选择的角度阈值（π/6）

    public:
        QuadGenerationModule(std::shared_ptr<Mesh> mesh,
            std::shared_ptr<FrontManagerModule> frontManager)
            : mesh(mesh), frontManager(frontManager), sideSelectionAngleThreshold(0.523599) {} // π/6

        // 设置侧边选择的角度阈值
        void setSideSelectionAngleThreshold(double threshold) {
            sideSelectionAngleThreshold = threshold;
        }

        // 从当前前沿边生成四边形
        bool generateQuadFromFrontEdge(const std::shared_ptr<Edge>& baseEdge) {
            // 1. 构建侧边
            std::shared_ptr<Edge> leftSide, rightSide;
            if (!buildSides(baseEdge, leftSide, rightSide)) {
                return false;
            }

            // 2. 构建顶边
            std::shared_ptr<Edge> topEdge;
            if (!buildTopEdge(leftSide, rightSide, topEdge)) {
                return false;
            }

            // 3. 形成四边形
            createQuad(baseEdge, leftSide, rightSide, topEdge);

            // 4. 更新前沿边队列
            updateFrontEdges(baseEdge, leftSide, rightSide, topEdge);

            return true;
        }

        // 构建侧边
        bool buildSides(const std::shared_ptr<Edge>& baseEdge,
            std::shared_ptr<Edge>& leftSide,
            std::shared_ptr<Edge>& rightSide) {
            // 获取基边的两个端点
            auto node1 = baseEdge->node1;
            auto node2 = baseEdge->node2;

            // 计算理想侧边方向（简化实现）
            Vector3D idealDirection1 = calculateIdealSideDirection(node1, baseEdge);
            Vector3D idealDirection2 = calculateIdealSideDirection(node2, baseEdge);

            // 1. 尝试利用现有边
            if (findExistingSideEdge(node1, idealDirection1, leftSide) &&
                findExistingSideEdge(node2, idealDirection2, rightSide)) {
                return true;
            }

            // 2. 尝试边翻转
            if (tryEdgeSwap(node1, idealDirection1, leftSide) &&
                tryEdgeSwap(node2, idealDirection2, rightSide)) {
                return true;
            }

            // 3. 尝试边分割
            if (tryEdgeSplit(node1, idealDirection1, leftSide) &&
                tryEdgeSplit(node2, idealDirection2, rightSide)) {
                return true;
            }

            return false;
        }

        // 计算理想侧边方向
        Vector3D calculateIdealSideDirection(const std::shared_ptr<Node>& node,
            const std::shared_ptr<Edge>& baseEdge) const {
            // 简化实现，实际需根据相邻前沿边计算角平分线
            return Vector3D(1, 0, 0); // 仅作示例
        }

        // 查找现有边作为侧边
        bool findExistingSideEdge(const std::shared_ptr<Node>& node,
            const Vector3D& idealDirection,
            std::shared_ptr<Edge>& sideEdge) {
            // 遍历节点的所有相邻边
            for (auto& edge : mesh->edges) {
                if (edge->node1 != node && edge->node2 != node) continue;

                // 计算边的方向
                Vector3D edgeDir = (edge->node1 == node) ?
                    (edge->node2->coordinates - node->coordinates).normalize() :
                    (edge->node1->coordinates - node->coordinates).normalize();

                // 计算与理想方向的夹角
                double angle = std::acos(edgeDir.dot(idealDirection));

                // 如果夹角小于阈值，选择此边
                if (angle < sideSelectionAngleThreshold) {
                    sideEdge = edge;
                    return true;
                }
            }

            return false;
        }

        // 尝试边翻转
        bool tryEdgeSwap(const std::shared_ptr<Node>& node,
            const Vector3D& idealDirection,
            std::shared_ptr<Edge>& sideEdge) {
            // 简化实现，实际需查找可翻转的边
            return false;
        }

        // 尝试边分割
        bool tryEdgeSplit(const std::shared_ptr<Node>& node,
            const Vector3D& idealDirection,
            std::shared_ptr<Edge>& sideEdge) {
            // 简化实现，实际需找到合适的边进行分割
            return false;
        }

        // 构建顶边
        bool buildTopEdge(const std::shared_ptr<Edge>& leftSide,
            const std::shared_ptr<Edge>& rightSide,
            std::shared_ptr<Edge>& topEdge) {
            // 获取两侧边的远端点
            auto leftNode = (leftSide->node1 == leftSide->node1) ? leftSide->node2 : leftSide->node1;
            auto rightNode = (rightSide->node1 == rightSide->node1) ? rightSide->node2 : rightSide->node1;

            // 创建顶边
            topEdge = mesh->addEdge(leftNode, rightNode);
            return true;
        }

        // 创建四边形
        void createQuad(const std::shared_ptr<Edge>& baseEdge,
            const std::shared_ptr<Edge>& leftSide,
            const std::shared_ptr<Edge>& rightSide,
            const std::shared_ptr<Edge>& topEdge) {
            // 获取四个顶点
            auto node1 = baseEdge->node1;
            auto node2 = baseEdge->node2;
            auto node3 = (leftSide->node1 == node1) ? leftSide->node2 : leftSide->node1;
            auto node4 = (rightSide->node1 == node2) ? rightSide->node2 : rightSide->node1;

            // 创建四边形
            auto quad = mesh->addQuad(node1, node2, node4, node3);

            // 添加四边形的边
            quad->edges.push_back(baseEdge);
            quad->edges.push_back(leftSide);
            quad->edges.push_back(rightSide);
            quad->edges.push_back(topEdge);

            // 标记内部三角形为已移除
            markInnerTrianglesForRemoval(node1, node2, node3, node4);

            // 清理已移除的三角形
            mesh->cleanupRemovedTriangles();
        }

        // 标记内部三角形为已移除
        void markInnerTrianglesForRemoval(const std::shared_ptr<Node>& n1,
            const std::shared_ptr<Node>& n2,
            const std::shared_ptr<Node>& n3,
            const std::shared_ptr<Node>& n4) {
            // 简化实现，实际需递归遍历内部三角形
            for (auto& triangle : mesh->triangles) {
                if (triangle->isRemoved) continue;

                // 检查三角形是否完全在四边形内部
                bool isInside = true;
                for (auto& node : triangle->nodes) {
                    if (node != n1 && node != n2 && node != n3 && node != n4) {
                        isInside = false;
                        break;
                    }
                }

                if (isInside) {
                    triangle->isRemoved = true;
                }
            }
        }

        // 更新前沿边队列
        void updateFrontEdges(const std::shared_ptr<Edge>& baseEdge,
            const std::shared_ptr<Edge>& leftSide,
            const std::shared_ptr<Edge>& rightSide,
            const std::shared_ptr<Edge>& topEdge) {
            // 基边不再是前沿边
            // 左侧边和右侧边可能需要更新状态
            // 顶边成为新的前沿边

            // 添加顶边到前沿边队列
            frontManager->addFrontEdge(topEdge);

            // 更新左侧边和右侧边的状态
            frontManager->updateFrontEdgeStatus(leftSide);
            frontManager->updateFrontEdgeStatus(rightSide);
        }
    };

} // namespace QMorph

#endif // QUAD_GENERATION_MODULE_HPP    