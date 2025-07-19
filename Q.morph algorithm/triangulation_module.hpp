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

    // �ߴ纯���ӿ�
    class SizeFunction {
    public:
        virtual double getSize(const Vector3D& point) const = 0;
        virtual ~SizeFunction() {}
    };

    // ���ȳߴ纯��ʵ��
    class UniformSizeFunction : public SizeFunction {
    private:
        double size;

    public:
        UniformSizeFunction(double size) : size(size) {}

        double getSize(const Vector3D& point) const override {
            return size;
        }
    };

    // ��������Ӧ�ߴ纯��ʵ��
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
            // ����Խ�󣬳ߴ�ԽС
            double normalizedCurvature = std::min(1.0, std::max(0.0, curvature));
            return minSize + (maxSize - minSize) * (1.0 - normalizedCurvature);
        }
    };

    // ������������ģ��
    class TriangulationModule {
    private:
        std::shared_ptr<Mesh> mesh;
        std::shared_ptr<SizeFunction> sizeFunction;
        double aspectRatioThreshold; // �ݺ����ֵ

    public:
        TriangulationModule(std::shared_ptr<Mesh> mesh)
            : mesh(mesh), aspectRatioThreshold(5.0) {}

        // ���óߴ纯��
        void setSizeFunction(std::shared_ptr<SizeFunction> sizeFunction) {
            this->sizeFunction = sizeFunction;
        }

        // �����ݺ����ֵ
        void setAspectRatioThreshold(double threshold) {
            aspectRatioThreshold = threshold;
        }

        // ������������
        void generateTriangulation(const std::vector<Vector3D>& boundaryPoints,
            const std::vector<std::vector<int>>& boundaryLoops) {
            if (!sizeFunction) {
                throw std::runtime_error("δ���óߴ纯��");
            }

            // 1. ��ʼ������ӱ߽��
            std::vector<std::shared_ptr<Node>> boundaryNodes;
            for (const auto& point : boundaryPoints) {
                auto node = mesh->addNode(point, true);
                boundaryNodes.push_back(node);
            }

            // 2. �����߽��
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

            // 3. ʹ���ƽ�ǰ�ط�������������
            // ��ʵ�֣�ʵ����ʹ���������ƽ�ǰ���㷨
            // �������ʾ��������һ���򵥵���������
            if (boundaryNodes.size() >= 3) {
                // ������ʼ������
                auto tri = mesh->addTriangle(boundaryNodes[0], boundaryNodes[1], boundaryNodes[2]);
                mesh->buildTriangleEdges(tri);
            }

            // 4. ϸ������������ߴ纯�����ݺ��Ҫ��
            refineMesh();

            // 5. ȷ�����������η�����һ��
            orientTriangles();
        }

        // ����ϸ��������ߴ纯�����ݺ��Ҫ��
        void refineMesh() {
            // ��ʵ�֣�ʵ����ݹ�ϸ��ֱ������ߴ���ݺ��Ҫ��
            bool needsRefinement = true;

            while (needsRefinement) {
                needsRefinement = false;

                // ���ÿ��������
                for (auto& triangle : mesh->triangles) {
                    if (triangle->isRemoved) continue;

                    // ��������γߴ���ݺ��
                    double size = calculateTriangleSize(triangle);
                    double aspectRatio = calculateTriangleAspectRatio(triangle);

                    // ��ȡ���������ĵ�Ŀ��ߴ�
                    Vector3D center = (triangle->nodes[0]->coordinates +
                        triangle->nodes[1]->coordinates +
                        triangle->nodes[2]->coordinates) * (1.0 / 3.0);
                    double targetSize = sizeFunction->getSize(center);

                    // ����ߴ����Ŀ��ߴ���ݺ�ȳ�����ֵ����ϸ��
                    if (size > targetSize * 1.2 || aspectRatio > aspectRatioThreshold) {
                        splitTriangle(triangle);
                        needsRefinement = true;
                        break;
                    }
                }
            }
        }

        // ���������γߴ�
        double calculateTriangleSize(const std::shared_ptr<Triangle>& triangle) const {
            // ��Ϊ�����α߳���ƽ��ֵ
            double edge1Length = triangle->edges[0]->length();
            double edge2Length = triangle->edges[1]->length();
            double edge3Length = triangle->edges[2]->length();
            return (edge1Length + edge2Length + edge3Length) / 3.0;
        }

        // �����������ݺ��
        double calculateTriangleAspectRatio(const std::shared_ptr<Triangle>& triangle) const {
            // ��ʵ�֣�ʵ�����������Բ�����Բ�뾶֮��
            double a = triangle->edges[0]->length();
            double b = triangle->edges[1]->length();
            double c = triangle->edges[2]->length();

            double s = (a + b + c) / 2.0;
            double area = std::sqrt(s * (s - a) * (s - b) * (s - c));

            if (area < 1e-10) return 1.0;

            // �ݺ�� = (���)^2 / (2 * ���)
            double maxEdge = std::max(a, std::max(b, c));
            return (maxEdge * maxEdge) / (2.0 * area);
        }

        // ϸ��������
        void splitTriangle(const std::shared_ptr<Triangle>& triangle) {
            // �ҵ����
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

            // ����ߵ��е�
            Vector3D midPoint = (node1->coordinates + node2->coordinates) * 0.5;

            // �����½ڵ�
            auto newNode = mesh->addNode(midPoint);

            // ������������
            for (int i = 0; i < 3; ++i) {
                if (i == longestEdgeIdx) continue;

                auto otherNode = triangle->nodes[i];
                auto tri1 = mesh->addTriangle(node1, newNode, otherNode);
                auto tri2 = mesh->addTriangle(newNode, node2, otherNode);

                mesh->buildTriangleEdges(tri1);
                mesh->buildTriangleEdges(tri2);
            }

            // ���ԭ������Ϊ���Ƴ�
            triangle->isRemoved = true;

            // �������Ƴ���������
            mesh->cleanupRemovedTriangles();
        }

        // ȷ�����������η�����һ��
        void orientTriangles() {
            // ��ʵ�֣�ʵ���账��������
            if (mesh->triangles.empty()) return;

            // ѡ���һ����������Ϊ�ο�
            auto referenceTriangle = mesh->triangles[0];
            Vector3D referenceNormal = referenceTriangle->normal;

            // �������������Σ�ȷ��������һ��
            for (auto& triangle : mesh->triangles) {
                if (!triangle->hasConsistentNormal(referenceNormal)) {
                    // ��ת�����ζ���˳��
                    std::swap(triangle->nodes[1], triangle->nodes[2]);

                    // ���¼��㷨����
                    Vector3D v1 = triangle->nodes[1]->coordinates - triangle->nodes[0]->coordinates;
                    Vector3D v2 = triangle->nodes[2]->coordinates - triangle->nodes[0]->coordinates;
                    triangle->normal = v1.cross(v2).normalize();
                }
            }
        }
    };

} // namespace QMorph

#endif // TRIANGULATION_MODULE_HPP    