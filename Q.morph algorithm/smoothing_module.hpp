#ifndef SMOOTHING_MODULE_HPP
#define SMOOTHING_MODULE_HPP

#include "mesh_structure.hpp"
#include <memory>
#include <vector>
#include <cmath>

namespace QMorph {

    // �ֲ�ƽ��ģ��
    class SmoothingModule {
    private:
        std::shared_ptr<Mesh> mesh;
        double boundaryEdgeLengthRatioThreshold; // �߽�߳��ȱ���ֵ

    public:
        SmoothingModule(std::shared_ptr<Mesh> mesh)
            : mesh(mesh), boundaryEdgeLengthRatioThreshold(2.5) {}

        // ���ñ߽�߳��ȱ���ֵ
        void setBoundaryEdgeLengthRatioThreshold(double threshold) {
            boundaryEdgeLengthRatioThreshold = threshold;
        }

        // ƽ������
        void smoothMesh() {
            // 1. ƽ���ڲ��ڵ�
            smoothInteriorNodes();

            // 2. ƽ��ǰ�ؽڵ�
            smoothFrontierNodes();

            // 3. ��鲢������������������
            checkAndFixNormalInversion();
        }

        // ƽ���ڲ��ڵ�
        void smoothInteriorNodes() {
            for (auto& node : mesh->nodes) {
                // �����߽�ڵ�
                if (node->isBoundary) continue;

                // �������ڽڵ������
                Vector3D centroid(0, 0, 0);
                int neighborCount = 0;

                // �������ڽڵ�
                std::vector<std::shared_ptr<Node>> neighbors;
                for (auto& edge : mesh->edges) {
                    if (edge->node1 == node) {
                        neighbors.push_back(edge->node2);
                    }
                    else if (edge->node2 == node) {
                        neighbors.push_back(edge->node1);
                    }
                }

                // ��������
                for (auto& neighbor : neighbors) {
                    centroid = centroid + neighbor->coordinates;
                    ++neighborCount;
                }

                if (neighborCount > 0) {
                    centroid = centroid * (1.0 / neighborCount);

                    // �ƶ��ڵ㵽����λ��
                    node->coordinates = centroid;
                }
            }
        }

        // ƽ��ǰ�ؽڵ�
        void smoothFrontierNodes() {
            // �������б�
            for (auto& edge : mesh->edges) {
                if (!edge->isBoundary) continue;

                // ����ߵ������˵�����ڱ߳��ȱ�
                double lengthRatio1 = calculateBoundaryEdgeLengthRatio(edge->node1);
                double lengthRatio2 = calculateBoundaryEdgeLengthRatio(edge->node2);

                // ���ݳ��ȱ�ѡ��ƽ������
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

        // ����߽�߳��ȱ�
        double calculateBoundaryEdgeLengthRatio(const std::shared_ptr<Node>& node) {
            // �����������ڵı߽��
            std::vector<std::shared_ptr<Edge>> boundaryEdges;
            for (auto& edge : mesh->edges) {
                if ((edge->node1 == node || edge->node2 == node) && edge->isBoundary) {
                    boundaryEdges.push_back(edge);
                }
            }

            // ���㳤�ȱ�
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

        // ʹ�õȲη���ƽ��ǰ�ؽڵ�
        void smoothFrontierNodeWithIsoParametric(const std::shared_ptr<Node>& node) {
            // ��ʵ�֣�ʵ����ʵ�ֵȲ�ƽ��+�Ƕ�����
            // �������ʾ����������ʵ��ƽ��
        }

        // ʹ�ø߼�����ƽ��ǰ�ؽڵ�
        void smoothFrontierNodeWithAdvanced(const std::shared_ptr<Node>& node) {
            // ��ʵ�֣�ʵ����������ڱ߳��Ⱥ�ǰ��״̬����Ŀ��λ��
            // �������ʾ����������ʵ��ƽ��
        }

        // ��鲢������������������
        void checkAndFixNormalInversion() {
            // �������нڵ�
            for (auto& node : mesh->nodes) {
                Vector3D originalPosition = node->coordinates;

                // ����ڵ��ƶ������λ��
                Vector3D newPosition = calculateSmoothedPosition(node);

                // ��ʱ�ƶ��ڵ㵽��λ��
                node->coordinates = newPosition;

                // ����Ƿ��з��������õ�������
                bool hasInvertedTriangles = checkForInvertedTriangles(node);

                if (hasInvertedTriangles) {
                    // �𲽵����ڵ�λ�ã�ֱ��û�е��õ�������
                    Vector3D direction = newPosition - originalPosition;
                    double step = 0.1; // ����

                    for (int i = 1; i <= 10; ++i) {
                        // ���Ը�С���ƶ�
                        Vector3D testPosition = originalPosition + direction * (step * i);
                        node->coordinates = testPosition;

                        if (!checkForInvertedTriangles(node)) {
                            // �ҵ����ʵ�λ��
                            break;
                        }

                        // ������������в������е��������Σ��ص�ԭʼλ��
                        if (i == 10) {
                            node->coordinates = originalPosition;
                        }
                    }
                }
            }
        }

        // ����ƽ����Ľڵ�λ��
        Vector3D calculateSmoothedPosition(const std::shared_ptr<Node>& node) {
            // ��ʵ�֣�ʵ�������ƽ���㷨����
            return node->coordinates;
        }

        // ���ڵ���Χ�Ƿ��з��������õ�������
        bool checkForInvertedTriangles(const std::shared_ptr<Node>& node) {
            // �������а����ýڵ��������
            for (auto& triangle : mesh->triangles) {
                if (triangle->isRemoved) continue;

                if (triangle->containsNode(node)) {
                    // ���������εķ�����
                    Vector3D v1 = triangle->nodes[1]->coordinates - triangle->nodes[0]->coordinates;
                    Vector3D v2 = triangle->nodes[2]->coordinates - triangle->nodes[0]->coordinates;
                    Vector3D normal = v1.cross(v2).normalize();

                    // ��鷨�����Ƿ���ԭʼ������һ��
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