#ifndef TRIANGULATION_MODULE_HPP
#define TRIANGULATION_MODULE_HPP

#include "mesh_structure.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <map>
#include <cmath>
#include <functional>
#include <stdexcept>

namespace QMorph {

    // 尺寸函数接口
    class SizeFunction {
    public:
        virtual double getSize(const Vector3D& point) const = 0;
        virtual ~SizeFunction() {}
    };

    // 均匀尺寸函数实现
    class UniformSizeFunction : public SizeFunction {
    private:
        double size;

    public:
        UniformSizeFunction(double size) : size(size) {}

        double getSize(const Vector3D& point) const override {
            return size;
        }
    };

    // 曲率自适应尺寸函数实现
    class CurvatureSizeFunction : public SizeFunction {
    private:
        double minSize;
        double maxSize;
        std::function<double(const Vector3D&)> curvatureFunction;

    public:
        CurvatureSizeFunction(double minSize, double maxSize,
            std::function<double(const Vector3D&)> curvatureFunction)
            : minSize(minSize), maxSize(maxSize), curvatureFunction(curvatureFunction) {}

        double getSize(const Vector3D& point) const override {
            double curvature = curvatureFunction(point);
            // 曲率越大，尺寸越小
            double normalizedCurvature = std::min(1.0, std::max(0.0, curvature));
            return minSize + (maxSize - minSize) * (1.0 - normalizedCurvature);
        }
    };

    // 三角网格生成模块
    class TriangulationModule {
    private:
        std::shared_ptr<Mesh> mesh;
        std::shared_ptr<SizeFunction> sizeFunction;
        double aspectRatioThreshold; // 纵横比阈值

    public:
        TriangulationModule(std::shared_ptr<Mesh> mesh)
            : mesh(mesh), aspectRatioThreshold(5.0) {}

        // 设置尺寸函数
        void setSizeFunction(std::shared_ptr<SizeFunction> sizeFunction) {
            this->sizeFunction = sizeFunction;
        }

        // 设置纵横比阈值
        void setAspectRatioThreshold(double threshold) {
            aspectRatioThreshold = threshold;
        }

        // 生成三角网格
        void generateTriangulation(const std::vector<Vector3D>& boundaryPoints,
            const std::vector<std::vector<int>>& boundaryLoops) {
            if (!sizeFunction) {
                throw std::runtime_error("未设置尺寸函数");
            }

            // 1. 初始化：添加边界点
            std::vector<std::shared_ptr<Node>> boundaryNodes;
            for (const auto& point : boundaryPoints) {
                auto node = mesh->addNode(point, true);
                boundaryNodes.push_back(node);
            }

            // 2. 构建边界边
            std::vector<std::shared_ptr<Edge>> boundaryEdges;
            for (const auto& loop : boundaryLoops) {
                for (size_t i = 0; i < loop.size(); ++i) {
                    int idx1 = loop[i];
                    int idx2 = loop[(i + 1) % loop.size()];
                    auto edge = mesh->addEdge(boundaryNodes[idx1], boundaryNodes[idx2]);
                    edge->isBoundary = true;
                    boundaryEdges.push_back(edge);
                }
            }

            // 3. 使用推进前沿法生成三角网格
            // 简化实现，实际需使用完整的推进前沿算法
            // 这里仅作示例，创建一个简单的三角网格
            if (boundaryNodes.size() >= 3) {
                // 创建初始三角形
                auto tri = mesh->addTriangle(boundaryNodes[0], boundaryNodes[1], boundaryNodes[2]);
                mesh->buildTriangleEdges(tri);
            }

            // 4. 细化网格以满足尺寸函数和纵横比要求
            refineMesh();

            // 5. 确保所有三角形法向量一致
            orientTriangles();
        }

        // 网格细化以满足尺寸函数和纵横比要求
        void refineMesh() {
            // 简化实现，实际需递归细分直到满足尺寸和纵横比要求
            bool needsRefinement = true;

            while (needsRefinement) {
                needsRefinement = false;

                // 检查每个三角形
                for (auto& triangle : mesh->triangles) {
                    if (triangle->isRemoved) continue;

                    // 检查三角形尺寸和纵横比
                    double size = calculateTriangleSize(triangle);
                    double aspectRatio = calculateTriangleAspectRatio(triangle);

                    // 获取三角形中心的目标尺寸
                    Vector3D center = (triangle->nodes[0]->coordinates +
                        triangle->nodes[1]->coordinates +
                        triangle->nodes[2]->coordinates) * (1.0 / 3.0);
                    double targetSize = sizeFunction->getSize(center);

                    // 如果尺寸大于目标尺寸或纵横比超过阈值，则细分
                    if (size > targetSize * 1.2 || aspectRatio > aspectRatioThreshold) {
                        splitTriangle(triangle);
                        needsRefinement = true;
                        break;
                    }
                }
            }
        }

        // 计算三角形尺寸
        double calculateTriangleSize(const std::shared_ptr<Triangle>& triangle) const {
            // 简化为三角形边长的平均值
            double edge1Length = triangle->edges[0]->length();
            double edge2Length = triangle->edges[1]->length();
            double edge3Length = triangle->edges[2]->length();
            return (edge1Length + edge2Length + edge3Length) / 3.0;
        }

        // 计算三角形纵横比
        double calculateTriangleAspectRatio(const std::shared_ptr<Triangle>& triangle) const {
            // 简化实现，实际需计算内切圆和外接圆半径之比
            double a = triangle->edges[0]->length();
            double b = triangle->edges[1]->length();
            double c = triangle->edges[2]->length();

            double s = (a + b + c) / 2.0;
            double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

            if (area < 1e-10) return 1.0;

            // 纵横比 = (最长边)^2 / (2 * 面积)
            double maxEdge = std::max(a, std::max(b, c));
            return (maxEdge * maxEdge) / (2.0 * area);
        }

        // 细分三角形
        void splitTriangle(const std::shared_ptr<Triangle>& triangle) {
            // 找到最长边
            int longestEdgeIdx = 0;
            double maxLength = triangle->edges[0]->length();

            for (int i = 1; i < 3; ++i) {
                double length = triangle->edges[i]->length();
                if (length > maxLength) {
                    maxLength = length;
                    longestEdgeIdx = i;
                }
            }

            auto longestEdge = triangle->edges[longestEdgeIdx];
            auto node1 = longestEdge->node1;
            auto node2 = longestEdge->node2;

            // 计算边的中点
            Vector3D midPoint = (node1->coordinates + node2->coordinates) * 0.5;

            // 创建新节点
            auto newNode = mesh->addNode(midPoint);

            // 创建新三角形
            for (int i = 0; i < 3; ++i) {
                if (i == longestEdgeIdx) continue;

                auto otherNode = triangle->nodes[i];
                auto tri1 = mesh->addTriangle(node1, newNode, otherNode);
                auto tri2 = mesh->addTriangle(newNode, node2, otherNode);

                mesh->buildTriangleEdges(tri1);
                mesh->buildTriangleEdges(tri2);
            }

            // 标记原三角形为已移除
            triangle->isRemoved = true;

            // 清理已移除的三角形
            mesh->cleanupRemovedTriangles();
        }

        // 确保所有三角形法向量一致
        void orientTriangles() {
            // 简化实现，实际需处理复杂曲面
            if (mesh->triangles.empty()) return;

            // 选择第一个三角形作为参考
            auto referenceTriangle = mesh->triangles[0];
            Vector3D referenceNormal = referenceTriangle->normal;

            // 遍历所有三角形，确保法向量一致
            for (auto& triangle : mesh->triangles) {
                if (!triangle->hasConsistentNormal(referenceNormal)) {
                    // 翻转三角形顶点顺序
                    std::swap(triangle->nodes[1], triangle->nodes[2]);

                    // 重新计算法向量
                    Vector3D v1 = triangle->nodes[1]->coordinates - triangle->nodes[0]->coordinates;
                    Vector3D v2 = triangle->nodes[2]->coordinates - triangle->nodes[0]->coordinates;
                    triangle->normal = v1.cross(v2).normalize();
                }
            }
        }
    };

} // namespace QMorph

#endif // TRIANGULATION_MODULE_HPP    