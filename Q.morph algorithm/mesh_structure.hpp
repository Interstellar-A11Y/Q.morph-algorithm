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

    // 3D�����࣬���ڱ�ʾ����ͷ�����
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

    // �ڵ���
    class Node {
    public:
        int id;
        Vector3D coordinates;
        bool isBoundary;
        bool isIrregular; // ������ڵ㣺����Ԫ������4

        Node(int id, const Vector3D& coords, bool boundary = false)
            : id(id), coordinates(coords), isBoundary(boundary), isIrregular(false) {}
    };

    // ����
    class Edge {
    public:
        int id;
        std::shared_ptr<Node> node1;
        std::shared_ptr<Node> node2;
        std::vector<std::shared_ptr<Triangle>> triangles; // �����������б�
        bool isBoundary;
        int status; // 0-0, 0-1, 1-0, 1-1
        int level;  // �㼶
        double priority; // ���ȼ�ֵ

        Edge(int id, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2)
            : id(id), node1(n1), node2(n2), isBoundary(false), status(0), level(0), priority(0.0) {
            triangles.reserve(2);
        }

        double length() const {
            return (node2->coordinates - node1->coordinates).magnitude();
        }

        // ����ߵķ�������
        Vector3D direction() const {
            return (node2->coordinates - node1->coordinates).normalize();
        }

        // �ж��������Ƿ���ڵ�
        bool sharesNode(const std::shared_ptr<Edge>& other) const {
            return node1 == other->node1 || node1 == other->node2 ||
                node2 == other->node1 || node2 == other->node2;
        }

        // ��ȡ����ڵ�
        std::shared_ptr<Node> getSharedNode(const std::shared_ptr<Edge>& other) const {
            if (node1 == other->node1 || node1 == other->node2) return node1;
            if (node2 == other->node1 || node2 == other->node2) return node2;
            return nullptr;
        }

        // ���±ߵ�״̬
        void updateStatus(double angleThreshold = 2.35619) { // 3��/4
            // ��������˵�������ǰ�رߵļн�
            // ��ʵ�֣�ʵ�����������ǰ�ر߼���
            double angle1 = 0.0; // ����ļн�1
            double angle2 = 0.0; // ����ļн�2

            status = ((angle1 < angleThreshold) ? 1 : 0) * 2 +
                ((angle2 < angleThreshold) ? 1 : 0);

            // �������ȼ�
            calculatePriority();
        }

        // ����ߵ����ȼ�
        void calculatePriority() {
            // ״̬���ȼ���1-1 > 0-1/1-0 > 0-0
            double statusPriority = 0.0;
            switch (status) {
            case 3: statusPriority = 3.0; break; // 1-1
            case 1: case 2: statusPriority = 2.0; break; // 0-1, 1-0
            case 0: statusPriority = 1.0; break; // 0-0
            }

            // �㼶Ȩ��
            double levelWeight = 1.0 / (level + 1);

            // ����Ȩ�� (�̱�����)
            double lengthWeight = 1.0 / (length() + 1e-10);

            // �ۺ����ȼ�����
            priority = statusPriority * 1000 + levelWeight * 100 + lengthWeight;
        }
    };

    // ��������
    class Triangle {
    public:
        int id;
        std::vector<std::shared_ptr<Node>> nodes; // 3���ڵ�
        std::vector<std::shared_ptr<Edge>> edges; // 3����
        Vector3D normal; // ������
        bool isRemoved; // ����Ƿ��Ƴ�

        Triangle(int id, std::shared_ptr<Node> n1, std::shared_ptr<Node> n2, std::shared_ptr<Node> n3)
            : id(id), isRemoved(false) {
            nodes.reserve(3);
            nodes.push_back(n1);
            nodes.push_back(n2);
            nodes.push_back(n3);
            edges.reserve(3);

            // ���㷨����
            Vector3D v1 = n2->coordinates - n1->coordinates;
            Vector3D v2 = n3->coordinates - n1->coordinates;
            normal = v1.cross(v2).normalize();
        }

        // ����������Ƿ����ĳ���ڵ�
        bool containsNode(const std::shared_ptr<Node>& node) const {
            return node == nodes[0] || node == nodes[1] || node == nodes[2];
        }

        // �������������
        double area() const {
            Vector3D v1 = nodes[1]->coordinates - nodes[0]->coordinates;
            Vector3D v2 = nodes[2]->coordinates - nodes[0]->coordinates;
            return v1.cross(v2).magnitude() / 2.0;
        }

        // ��������η������Ƿ���Ŀ�귨����һ��
        bool hasConsistentNormal(const Vector3D& targetNormal, double threshold = 0.0) const {
            return normal.dot(targetNormal) > threshold;
        }
    };

    // �ı�����
    class Quad {
    public:
        int id;
        std::vector<std::shared_ptr<Node>> nodes; // 4���ڵ�
        std::vector<std::shared_ptr<Edge>> edges; // 4����
        bool isBoundary;
        double quality; // ����ָ���

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

        // �����ı�������ָ���
        void calculateQuality() {
            // ��ʵ�֣�ʵ��������ı�����״��������ָ��
            // �� = min(��_i/90��, 180��/��_i) ���Ц�_i���ı����ڽ�
            double minAngle = 360.0;
            double maxAngle = 0.0;

            // �������ʾ����ʵ����Ҫ�����ĸ��ڽ�
            for (int i = 0; i < 4; ++i) {
                double angle = 90.0; // �������нǶ���90��
                minAngle = std::min(minAngle, angle);
                maxAngle = std::max(maxAngle, angle);
            }

            quality = std::min(minAngle / 90.0, 180.0 / maxAngle);
        }
    };

    // �����࣬������������Ԫ��
    class Mesh {
    public:
        std::vector<std::shared_ptr<Node>> nodes;
        std::vector<std::shared_ptr<Edge>> edges;
        std::vector<std::shared_ptr<Triangle>> triangles;
        std::vector<std::shared_ptr<Quad>> quads;

        // �ڵ�ID���ڵ��ӳ��
        std::map<int, std::shared_ptr<Node>> nodeMap;
        // ��ID���ߵ�ӳ��
        std::map<int, std::shared_ptr<Edge>> edgeMap;
        // ������ID�������ε�ӳ��
        std::map<int, std::shared_ptr<Triangle>> triangleMap;
        // �ı���ID���ı��ε�ӳ��
        std::map<int, std::shared_ptr<Quad>> quadMap;

        // ��ӽڵ�
        std::shared_ptr<Node> addNode(const Vector3D& coords, bool boundary = false) {
            int id = nodes.size();
            auto node = std::make_shared<Node>(id, coords, boundary);
            nodes.push_back(node);
            nodeMap[id] = node;
            return node;
        }

        // ��ӱ�
        std::shared_ptr<Edge> addEdge(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) {
            int id = edges.size();
            auto edge = std::make_shared<Edge>(id, n1, n2);
            edges.push_back(edge);
            edgeMap[id] = edge;
            return edge;
        }

        // ���������
        std::shared_ptr<Triangle> addTriangle(std::shared_ptr<Node> n1,
            std::shared_ptr<Node> n2,
            std::shared_ptr<Node> n3) {
            int id = triangles.size();
            auto triangle = std::make_shared<Triangle>(id, n1, n2, n3);
            triangles.push_back(triangle);
            triangleMap[id] = triangle;
            return triangle;
        }

        // ����ı���
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

        // ���һ򴴽���
        std::shared_ptr<Edge> findOrCreateEdge(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2) {
            // �����Ƿ��Ѵ���
            for (auto& edge : edges) {
                if ((edge->node1 == n1 && edge->node2 == n2) ||
                    (edge->node1 == n2 && edge->node2 == n1)) {
                    return edge;
                }
            }

            // ���������򴴽��±�
            return addEdge(n1, n2);
        }

        // ���������εı�
        void buildTriangleEdges(std::shared_ptr<Triangle> triangle) {
            // ȷ�������εı��Ѵ���������
            for (int i = 0; i < 3; ++i) {
                auto n1 = triangle->nodes[i];
                auto n2 = triangle->nodes[(i + 1) % 3];
                auto edge = findOrCreateEdge(n1, n2);

                // ����������ӵ��ߵ������������б�
                if (std::find(edge->triangles.begin(), edge->triangles.end(), triangle) == edge->triangles.end()) {
                    edge->triangles.push_back(triangle);
                }

                // ������ӵ������εı��б�
                if (std::find(triangle->edges.begin(), triangle->edges.end(), edge) == triangle->edges.end()) {
                    triangle->edges.push_back(edge);
                }
            }

            // ���±ߵı߽�״̬
            for (auto& edge : triangle->edges) {
                edge->isBoundary = (edge->triangles.size() == 1);
            }
        }

        // ������Ϊ���Ƴ���������
        void cleanupRemovedTriangles() {
            std::vector<std::shared_ptr<Triangle>> remainingTriangles;
            for (auto& triangle : triangles) {
                if (!triangle->isRemoved) {
                    remainingTriangles.push_back(triangle);
                }
            }
            triangles = remainingTriangles;

            // ����������ӳ��
            triangleMap.clear();
            for (size_t i = 0; i < triangles.size(); ++i) {
                triangles[i]->id = i;
                triangleMap[i] = triangles[i];
            }
        }

        // ����ڵ������Ԫ����
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

        // ʶ�𲻹���ڵ�
        void identifyIrregularNodes() {
            for (auto& node : nodes) {
                int neighborCount = countNodeNeighbors(node);
                node->isIrregular = (neighborCount != 4);
            }
        }

        // ������������ͳ����Ϣ
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

            // ���㲻����ڵ�����
            identifyIrregularNodes();
            quality.irregularNodeCount = 0;
            for (auto& node : nodes) {
                if (node->isIrregular) {
                    ++quality.irregularNodeCount;
                }
            }

            return quality;
        }

        // ���ΪVTK��ʽ
        void exportToVTK(const std::string& filename) const {
            std::ofstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("�޷����ļ�: " + filename);
            }

            file << "# vtk DataFile Version 3.0\n";
            file << "Q-Morph Mesh\n";
            file << "ASCII\n";
            file << "DATASET UNSTRUCTURED_GRID\n";

            // д��ڵ�
            file << "POINTS " << nodes.size() << " double\n";
            for (const auto& node : nodes) {
                file << node->coordinates.x << " "
                    << node->coordinates.y << " "
                    << node->coordinates.z << "\n";
            }

            // д�뵥Ԫ
            size_t cellCount = triangles.size() + quads.size();
            file << "\nCELLS " << cellCount << " " << (cellCount + triangles.size() * 4 + quads.size() * 5) << "\n";

            // д��������
            for (const auto& triangle : triangles) {
                if (triangle->isRemoved) continue;
                file << "3 " << triangle->nodes[0]->id << " "
                    << triangle->nodes[1]->id << " "
                    << triangle->nodes[2]->id << "\n";
            }

            // д���ı���
            for (const auto& quad : quads) {
                file << "4 " << quad->nodes[0]->id << " "
                    << quad->nodes[1]->id << " "
                    << quad->nodes[2]->id << " "
                    << quad->nodes[3]->id << "\n";
            }

            // д�뵥Ԫ����
            file << "\nCELL_TYPES " << cellCount << "\n";
            for (size_t i = 0; i < triangles.size(); ++i) {
                if (triangles[i]->isRemoved) continue;
                file << "5\n"; // VTK_TRIANGLE
            }
            for (size_t i = 0; i < quads.size(); ++i) {
                file << "9\n"; // VTK_QUAD
            }

            // д��ڵ�����
            file << "\nPOINT_DATA " << nodes.size() << "\n";

            // д��ڵ��Ƿ�Ϊ�߽�ڵ�
            file << "SCALARS is_boundary int 1\n";
            file << "LOOKUP_TABLE default\n";
            for (const auto& node : nodes) {
                file << (node->isBoundary ? 1 : 0) << "\n";
            }

            // д��ڵ��Ƿ�Ϊ������ڵ�
            file << "\nSCALARS is_irregular int 1\n";
            file << "LOOKUP_TABLE default\n";
            for (const auto& node : nodes) {
                file << (node->isIrregular ? 1 : 0) << "\n";
            }

            // ������ı��Σ�д���ı�������
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