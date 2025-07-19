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

    // �ı�������ģ��
    class QuadGenerationModule {
    private:
        std::shared_ptr<Mesh> mesh;
        std::shared_ptr<FrontManagerModule> frontManager;
        double sideSelectionAngleThreshold; // ���ѡ��ĽǶ���ֵ����/6��

    public:
        QuadGenerationModule(std::shared_ptr<Mesh> mesh,
            std::shared_ptr<FrontManagerModule> frontManager)
            : mesh(mesh), frontManager(frontManager), sideSelectionAngleThreshold(0.523599) {} // ��/6

        // ���ò��ѡ��ĽǶ���ֵ
        void setSideSelectionAngleThreshold(double threshold) {
            sideSelectionAngleThreshold = threshold;
        }

        // �ӵ�ǰǰ�ر������ı���
        bool generateQuadFromFrontEdge(const std::shared_ptr<Edge>& baseEdge) {
            // 1. �������
            std::shared_ptr<Edge> leftSide, rightSide;
            if (!buildSides(baseEdge, leftSide, rightSide)) {
                return false;
            }

            // 2. ��������
            std::shared_ptr<Edge> topEdge;
            if (!buildTopEdge(leftSide, rightSide, topEdge)) {
                return false;
            }

            // 3. �γ��ı���
            createQuad(baseEdge, leftSide, rightSide, topEdge);

            // 4. ����ǰ�ر߶���
            updateFrontEdges(baseEdge, leftSide, rightSide, topEdge);

            return true;
        }

        // �������
        bool buildSides(const std::shared_ptr<Edge>& baseEdge,
            std::shared_ptr<Edge>& leftSide,
            std::shared_ptr<Edge>& rightSide) {
            // ��ȡ���ߵ������˵�
            auto node1 = baseEdge->node1;
            auto node2 = baseEdge->node2;

            // ���������߷��򣨼�ʵ�֣�
            Vector3D idealDirection1 = calculateIdealSideDirection(node1, baseEdge);
            Vector3D idealDirection2 = calculateIdealSideDirection(node2, baseEdge);

            // 1. �����������б�
            if (findExistingSideEdge(node1, idealDirection1, leftSide) &&
                findExistingSideEdge(node2, idealDirection2, rightSide)) {
                return true;
            }

            // 2. ���Ա߷�ת
            if (tryEdgeSwap(node1, idealDirection1, leftSide) &&
                tryEdgeSwap(node2, idealDirection2, rightSide)) {
                return true;
            }

            // 3. ���Ա߷ָ�
            if (tryEdgeSplit(node1, idealDirection1, leftSide) &&
                tryEdgeSplit(node2, idealDirection2, rightSide)) {
                return true;
            }

            return false;
        }

        // ���������߷���
        Vector3D calculateIdealSideDirection(const std::shared_ptr<Node>& node,
            const std::shared_ptr<Edge>& baseEdge) const {
            // ��ʵ�֣�ʵ�����������ǰ�ر߼����ƽ����
            return Vector3D(1, 0, 0); // ����ʾ��
        }

        // �������б���Ϊ���
        bool findExistingSideEdge(const std::shared_ptr<Node>& node,
            const Vector3D& idealDirection,
            std::shared_ptr<Edge>& sideEdge) {
            // �����ڵ���������ڱ�
            for (auto& edge : mesh->edges) {
                if (edge->node1 != node && edge->node2 != node) continue;

                // ����ߵķ���
                Vector3D edgeDir = (edge->node1 == node) ?
                    (edge->node2->coordinates - node->coordinates).normalize() :
                    (edge->node1->coordinates - node->coordinates).normalize();

                // ���������뷽��ļн�
                double angle = std::acos(edgeDir.dot(idealDirection));

                // ����н�С����ֵ��ѡ��˱�
                if (angle < sideSelectionAngleThreshold) {
                    sideEdge = edge;
                    return true;
                }
            }

            return false;
        }

        // ���Ա߷�ת
        bool tryEdgeSwap(const std::shared_ptr<Node>& node,
            const Vector3D& idealDirection,
            std::shared_ptr<Edge>& sideEdge) {
            // ��ʵ�֣�ʵ������ҿɷ�ת�ı�
            return false;
        }

        // ���Ա߷ָ�
        bool tryEdgeSplit(const std::shared_ptr<Node>& node,
            const Vector3D& idealDirection,
            std::shared_ptr<Edge>& sideEdge) {
            // ��ʵ�֣�ʵ�����ҵ����ʵı߽��зָ�
            return false;
        }

        // ��������
        bool buildTopEdge(const std::shared_ptr<Edge>& leftSide,
            const std::shared_ptr<Edge>& rightSide,
            std::shared_ptr<Edge>& topEdge) {
            // ��ȡ����ߵ�Զ�˵�
            auto leftNode = (leftSide->node1 == leftSide->node1) ? leftSide->node2 : leftSide->node1;
            auto rightNode = (rightSide->node1 == rightSide->node1) ? rightSide->node2 : rightSide->node1;

            // ��������
            topEdge = mesh->addEdge(leftNode, rightNode);
            return true;
        }

        // �����ı���
        void createQuad(const std::shared_ptr<Edge>& baseEdge,
            const std::shared_ptr<Edge>& leftSide,
            const std::shared_ptr<Edge>& rightSide,
            const std::shared_ptr<Edge>& topEdge) {
            // ��ȡ�ĸ�����
            auto node1 = baseEdge->node1;
            auto node2 = baseEdge->node2;
            auto node3 = (leftSide->node1 == node1) ? leftSide->node2 : leftSide->node1;
            auto node4 = (rightSide->node1 == node2) ? rightSide->node2 : rightSide->node1;

            // �����ı���
            auto quad = mesh->addQuad(node1, node2, node4, node3);

            // ����ı��εı�
            quad->edges.push_back(baseEdge);
            quad->edges.push_back(leftSide);
            quad->edges.push_back(rightSide);
            quad->edges.push_back(topEdge);

            // ����ڲ�������Ϊ���Ƴ�
            markInnerTrianglesForRemoval(node1, node2, node3, node4);

            // �������Ƴ���������
            mesh->cleanupRemovedTriangles();
        }

        // ����ڲ�������Ϊ���Ƴ�
        void markInnerTrianglesForRemoval(const std::shared_ptr<Node>& n1,
            const std::shared_ptr<Node>& n2,
            const std::shared_ptr<Node>& n3,
            const std::shared_ptr<Node>& n4) {
            // ��ʵ�֣�ʵ����ݹ�����ڲ�������
            for (auto& triangle : mesh->triangles) {
                if (triangle->isRemoved) continue;

                // ����������Ƿ���ȫ���ı����ڲ�
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

        // ����ǰ�ر߶���
        void updateFrontEdges(const std::shared_ptr<Edge>& baseEdge,
            const std::shared_ptr<Edge>& leftSide,
            const std::shared_ptr<Edge>& rightSide,
            const std::shared_ptr<Edge>& topEdge) {
            // ���߲�����ǰ�ر�
            // ���ߺ��Ҳ�߿�����Ҫ����״̬
            // ���߳�Ϊ�µ�ǰ�ر�

            // ��Ӷ��ߵ�ǰ�ر߶���
            frontManager->addFrontEdge(topEdge);

            // �������ߺ��Ҳ�ߵ�״̬
            frontManager->updateFrontEdgeStatus(leftSide);
            frontManager->updateFrontEdgeStatus(rightSide);
        }
    };

} // namespace QMorph

#endif // QUAD_GENERATION_MODULE_HPP    