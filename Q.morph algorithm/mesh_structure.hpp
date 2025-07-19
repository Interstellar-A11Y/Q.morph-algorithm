#ifndef MESH_STRUCTURE_HPP
#define MESH_STRUCTURE_HPP

#include <vector>
#include <memory>
#include <cmath>
#include <queue>
#include <map>
#include <set>
#include <algorithm>
#include <functional>
#include <fstream>
#include <sstream>
#include <string>
#include <stdexcept>

namespace QMorph {

    // 3D向量类，用于表示坐标和法向量
    class Vector3D {
    public:
        double x, y, z;

        Vector3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

        Vector3D operator+(const Vector3D& v) const {
            return Vector3D(x + v.x, y + v.y, z + v.z);
        }

        Vector3D operator-(const Vector3D& v) const {
            return Vector3D(x - v.x, y - v.y, z - v.z);
        }

        Vector3D operator*(double scalar) const {
            return Vector3D(x * scalar, y * scalar, z * scalar);
        }

        double dot(const Vector3D& v) const {
            return x * v.x + y * v.y + z * v.z;
        }

        Vector3D cross(const Vector3D& v) const {
            return Vector3D(
                y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x
            );
        }

        double magnitude() const {
            return std::sqrt(x * x + y * y + z * z);
        }

        Vector3D normalize() const {
            double mag = magnitude();
            if (mag < 1e-10) return Vector3D();
            return Vector3D(x / mag, y / mag, z / mag);
        }
    };

    // 节点类
    class Node {
    public:
        int id;
        Vector3D coordinates;
        bool isBoundary;
        bool isIrregular; // 不规则节点：相邻元素数≠4

        Node(int id, const Vector3D& coords, bool boundary = false)
            : id(id), coordinates(coords), isBoundary(boundary), isIrregular(false) {}
    };

    // 边类
    class Edge {
    public:
        int id;
        std::shared_ptr<Node> node1;
        std::shared_ptr<Node> node2;
        std::vector<std::shared_ptr<Triangle>> triangles; // 相邻三角形列表
        bool isBoundary;
        int status; // 0-0, 0-1, 1-0, 1-1
        int level;  // 层级
        double priority; // 优先级值

        Edge(int id, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2)
            : id(id), node1(n1), node2(n2), isBoundary(false), status(0), level(0), priority(0.0) {
            triangles.reserve(2);
        }

        double length() const {
            return (node2->coordinates - node1->coordinates).magnitude();
        }

        // 计算边的方向向量
        Vector3D direction() const {
            return (node2->coordinates - node1->coordinates).normalize();
        }

        // 判断两条边是否共享节点
        bool sharesNode(const std::shared_ptr<Edge>& other) const {
            return node1 == other->node1 || node1 == other->node2 ||
                node2 == other->node1 || node2 == other->node2;
        }

        // 获取共享节点
        std::shared_ptr<Node> getSharedNode(const std::shared_ptr<Edge>& other) const {
            if (node1 == other->node1 || node1 == other->node2) return node1;
            if (node2 == other->node1 || node2 == other->node2) return node2;
            return nullptr;
        }

        // 更新边的状态
        void updateStatus(double angleThreshold = 2.35619) { // 3π/4
            // 计算边两端点与相邻前沿边的夹角
            // 简化实现，实际需根据相邻前沿边计算
            double angle1 = 0.0; // 假设的夹角1
            double angle2 = 0.0; // 假设的夹角2

            status = ((angle1 < angleThreshold) ? 1 : 0) * 2 +
                ((angle2 < angleThreshold) ? 1 : 0);

            // 计算优先级
            calculatePriority();
        }

        // 计算边的优先级
        void calculatePriority() {
            // 状态优先级：1-1 > 0-1/1-0 > 0-0
            double statusPriority = 0.0;
            switch (status) {
            case 3: statusPriority = 3.0; break; // 1-1
            case 1: case 2: statusPriority = 2.0; break; // 0-1, 1-0
            case 0: statusPriority = 1.0; break; // 0-0
            }

            // 层级权重
            double levelWeight = 1.0 / (level + 1);

            // 长度权重 (短边优先)
            double lengthWeight = 1.0 / (length() + 1e-10);

            // 综合优先级计算
            priority = statusPriority * 1000 + levelWeight * 100 + lengthWeight;
        }
    };

    // 三角形类
    class Triangle {
    public:
        int id;
        std::vector<std::shared_ptr<Node>> nodes; // 3个节点
        std::vector<std::shared_ptr<Edge>> edges; // 3条边
        Vector3D normal; // 法向量
        bool isRemoved; // 标记是否被移除

        Triangle(int id, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, std::shared_ptr<Node> n3)
            : id(id), isRemoved(false) {
            nodes.reserve(3);
            nodes.push_back(n1);
            nodes.push_back(n2);
            nodes.push_back(n3);
            edges.reserve(3);

            // 计算法向量
            Vector3D v1 = n2->coordinates - n1->coordinates;
            Vector3D v2 = n3->coordinates - n1->coordinates;
            normal = v1.cross(v2).normalize();
        }

        // 检查三角形是否包含某个节点
        bool containsNode(const std::shared_ptr<Node>& node) const {
            return node == nodes[0] || node == nodes[1] || node == nodes[2];
        }

        // 计算三角形面积
        double area() const {
            Vector3D v1 = nodes[1]->coordinates - nodes[0]->coordinates;
            Vector3D v2 = nodes[2]->coordinates - nodes[0]->coordinates;
            return v1.cross(v2).magnitude() / 2.0;
        }

        // 检查三角形法向量是否与目标法向量一致
        bool hasConsistentNormal(const Vector3D& targetNormal, double threshold = 0.0) const {
            return normal.dot(targetNormal) > threshold;
        }
    };

    // 四边形类
    class Quad {
    public:
        int id;
        std::vector<std::shared_ptr<Node>> nodes; // 4个节点
        std::vector<std::shared_ptr<Edge>> edges; // 4条边
        bool isBoundary;
        double quality; // 质量指标β

        Quad(int id, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2,
            std::shared_ptr<Node> n3, std::shared_ptr<Node> n4)
            : id(id), isBoundary(false), quality(0.0) {
            nodes.reserve(4);
            nodes.push_back(n1);
            nodes.push_back(n2);
            nodes.push_back(n3);
            nodes.push_back(n4);
            edges.reserve(4);
        }

        // 计算四边形质量指标β
        void calculateQuality() {
            // 简化实现，实际需根据四边形形状计算质量指标
            // β = min(θ_i/90°, 180°/θ_i) 其中θ_i是四边形内角
            double minAngle = 360.0;
            double maxAngle = 0.0;

            // 这里仅作示例，实际需要计算四个内角
            for (int i = 0; i < 4; ++i) {
                double angle = 90.0; // 假设所有角都是90度
                minAngle = std::min(minAngle, angle);
                maxAngle = std::max(maxAngle, angle);
            }

            quality = std::min(minAngle / 90.0, 180.0 / maxAngle);
        }
    };

    // 网格类，管理所有网格元素
    class Mesh {
    public:
        std::vector<std::shared_ptr<Node>> nodes;
        std::vector<std::shared_ptr<Edge>> edges;
        std::vector<std::shared_ptr<Triangle>> triangles;
        std::vector<std::shared_ptr<Quad>> quads;

        // 节点ID到节点的映射
        std::map<int, std::shared_ptr<Node>> nodeMap;
        // 边ID到边的映射
        std::map<int, std::shared_ptr<Edge>> edgeMap;
        // 三角形ID到三角形的映射
        std::map<int, std::shared_ptr<Triangle>> triangleMap;
        // 四边形ID到四边形的映射
        std::map<int, std::shared_ptr<Quad>> quadMap;

        // 添加节点
        std::shared_ptr<Node> addNode(const Vector3D& coords, bool boundary = false) {
            int id = nodes.size();
            auto node = std::make_shared<Node>(id, coords, boundary);
            nodes.push_back(node);
            nodeMap[id] = node;
            return node;
        }

        // 添加边
        std::shared_ptr<Edge> addEdge(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) {
            int id = edges.size();
            auto edge = std::make_shared<Edge>(id, n1, n2);
            edges.push_back(edge);
            edgeMap[id] = edge;
            return edge;
        }

        // 添加三角形
        std::shared_ptr<Triangle> addTriangle(std::shared_ptr<Node> n1,
            std::shared_ptr<Node> n2,
            std::shared_ptr<Node> n3) {
            int id = triangles.size();
            auto triangle = std::make_shared<Triangle>(id, n1, n2, n3);
            triangles.push_back(triangle);
            triangleMap[id] = triangle;
            return triangle;
        }

        // 添加四边形
        std::shared_ptr<Quad> addQuad(std::shared_ptr<Node> n1,
            std::shared_ptr<Node> n2,
            std::shared_ptr<Node> n3,
            std::shared_ptr<Node> n4) {
            int id = quads.size();
            auto quad = std::make_shared<Quad>(id, n1, n2, n3, n4);
            quads.push_back(quad);
            quadMap[id] = quad;
            return quad;
        }

        // 查找或创建边
        std::shared_ptr<Edge> findOrCreateEdge(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) {
            // 检查边是否已存在
            for (auto& edge : edges) {
                if ((edge->node1 == n1 && edge->node2 == n2) ||
                    (edge->node1 == n2 && edge->node2 == n1)) {
                    return edge;
                }
            }

            // 若不存在则创建新边
            return addEdge(n1, n2);
        }

        // 构建三角形的边
        void buildTriangleEdges(std::shared_ptr<Triangle> triangle) {
            // 确保三角形的边已创建并关联
            for (int i = 0; i < 3; ++i) {
                auto n1 = triangle->nodes[i];
                auto n2 = triangle->nodes[(i + 1) % 3];
                auto edge = findOrCreateEdge(n1, n2);

                // 将三角形添加到边的相邻三角形列表
                if (std::find(edge->triangles.begin(), edge->triangles.end(), triangle) == edge->triangles.end()) {
                    edge->triangles.push_back(triangle);
                }

                // 将边添加到三角形的边列表
                if (std::find(triangle->edges.begin(), triangle->edges.end(), edge) == triangle->edges.end()) {
                    triangle->edges.push_back(edge);
                }
            }

            // 更新边的边界状态
            for (auto& edge : triangle->edges) {
                edge->isBoundary = (edge->triangles.size() == 1);
            }
        }

        // 清除标记为已移除的三角形
        void cleanupRemovedTriangles() {
            std::vector<std::shared_ptr<Triangle>> remainingTriangles;
            for (auto& triangle : triangles) {
                if (!triangle->isRemoved) {
                    remainingTriangles.push_back(triangle);
                }
            }
            triangles = remainingTriangles;

            // 更新三角形映射
            triangleMap.clear();
            for (size_t i = 0; i < triangles.size(); ++i) {
                triangles[i]->id = i;
                triangleMap[i] = triangles[i];
            }
        }

        // 计算节点的相邻元素数
        int countNodeNeighbors(const std::shared_ptr<Node>& node) const {
            int count = 0;
            for (auto& triangle : triangles) {
                if (triangle->containsNode(node) && !triangle->isRemoved) {
                    ++count;
                }
            }
            for (auto& quad : quads) {
                if (std::find(quad->nodes.begin(), quad->nodes.end(), node) != quad->nodes.end()) {
                    ++count;
                }
            }
            return count;
        }

        // 识别不规则节点
        void identifyIrregularNodes() {
            for (auto& node : nodes) {
                int neighborCount = countNodeNeighbors(node);
                node->isIrregular = (neighborCount != 4);
            }
        }

        // 计算网格质量统计信息
        struct MeshQuality {
            double avgQuality;
            double minQuality;
            double maxQuality;
            int irregularNodeCount;

            MeshQuality() : avgQuality(0.0), minQuality(1.0), maxQuality(0.0), irregularNodeCount(0) {}
        };

        MeshQuality calculateMeshQuality() {
            MeshQuality quality;

            if (quads.empty()) return quality;

            double totalQuality = 0.0;
            quality.minQuality = 1.0;
            quality.maxQuality = 0.0;

            for (auto& quad : quads) {
                quad->calculateQuality();
                totalQuality += quad->quality;
                quality.minQuality = std::min(quality.minQuality, quad->quality);
                quality.maxQuality = std::max(quality.maxQuality, quad->quality);
            }

            quality.avgQuality = totalQuality / quads.size();

            // 计算不规则节点数量
            identifyIrregularNodes();
            quality.irregularNodeCount = 0;
            for (auto& node : nodes) {
                if (node->isIrregular) {
                    ++quality.irregularNodeCount;
                }
            }

            return quality;
        }

        // 输出为VTK格式
        void exportToVTK(const std::string& filename) const {
            std::ofstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("无法打开文件: " + filename);
            }

            file << "# vtk DataFile Version 3.0\n";
            file << "Q-Morph Mesh\n";
            file << "ASCII\n";
            file << "DATASET UNSTRUCTURED_GRID\n";

            // 写入节点
            file << "POINTS " << nodes.size() << " double\n";
            for (const auto& node : nodes) {
                file << node->coordinates.x << " "
                    << node->coordinates.y << " "
                    << node->coordinates.z << "\n";
            }

            // 写入单元
            size_t cellCount = triangles.size() + quads.size();
            file << "\nCELLS " << cellCount << " " << (cellCount + triangles.size() * 4 + quads.size() * 5) << "\n";

            // 写入三角形
            for (const auto& triangle : triangles) {
                if (triangle->isRemoved) continue;
                file << "3 " << triangle->nodes[0]->id << " "
                    << triangle->nodes[1]->id << " "
                    << triangle->nodes[2]->id << "\n";
            }

            // 写入四边形
            for (const auto& quad : quads) {
                file << "4 " << quad->nodes[0]->id << " "
                    << quad->nodes[1]->id << " "
                    << quad->nodes[2]->id << " "
                    << quad->nodes[3]->id << "\n";
            }

            // 写入单元类型
            file << "\nCELL_TYPES " << cellCount << "\n";
            for (size_t i = 0; i < triangles.size(); ++i) {
                if (triangles[i]->isRemoved) continue;
                file << "5\n"; // VTK_TRIANGLE
            }
            for (size_t i = 0; i < quads.size(); ++i) {
                file << "9\n"; // VTK_QUAD
            }

            // 写入节点数据
            file << "\nPOINT_DATA " << nodes.size() << "\n";

            // 写入节点是否为边界节点
            file << "SCALARS is_boundary int 1\n";
            file << "LOOKUP_TABLE default\n";
            for (const auto& node : nodes) {
                file << (node->isBoundary ? 1 : 0) << "\n";
            }

            // 写入节点是否为不规则节点
            file << "\nSCALARS is_irregular int 1\n";
            file << "LOOKUP_TABLE default\n";
            for (const auto& node : nodes) {
                file << (node->isIrregular ? 1 : 0) << "\n";
            }

            // 如果有四边形，写入四边形质量
            if (!quads.empty()) {
                file << "\nCELL_DATA " << quads.size() << "\n";
                file << "SCALARS quality double 1\n";
                file << "LOOKUP_TABLE default\n";
                for (const auto& quad : quads) {
                    file << quad->quality << "\n";
                }
            }

            file.close();
        }
    };

} // namespace QMorph

#endif // MESH_STRUCTURE_HPP    