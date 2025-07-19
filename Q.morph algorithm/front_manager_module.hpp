#ifndef FRONT_MANAGER_MODULE_HPP
#define FRONT_MANAGER_MODULE_HPP

#include "mesh_structure.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <map>
#include <functional>

namespace QMorph {

    // ǰ�ر߱Ƚ������������ȶ���
    struct FrontEdgeComparator {
        bool operator()(const std::shared_ptr<Edge>& e1, const std::shared_ptr<Edge>& e2) const {
            return e1->priority < e2->priority;
        }
    };

    // ǰ�ر߹���ģ��
    class FrontManagerModule {
    private:
        std::shared_ptr<Mesh> mesh;
        std::priority_queue<std::shared_ptr<Edge>, std::vector<std::shared_ptr<Edge>>, FrontEdgeComparator> frontQueue;
        std::map<int, std::shared_ptr<Edge>> activeFrontEdges; // ID��ǰ�رߵ�ӳ��
        double angleThreshold; // ״̬����ĽǶ���ֵ

    public:
        FrontManagerModule(std::shared_ptr<Mesh> mesh)
            : mesh(mesh), angleThreshold(2.35619) {} // 3��/4

        // ��ʼ��ǰ�ر߶���
        void initializeFrontEdges() {
            // ������ж���
            while (!frontQueue.empty()) {
                frontQueue.pop();
            }
            activeFrontEdges.clear();

            // �������бߣ��ҳ��߽�ߣ�������1�������εıߣ�
            for (auto& edge : mesh->edges) {
                if (edge->triangles.size() == 1) {
                    // ���±ߵ�״̬
                    edge->updateStatus(angleThreshold);

                    // ��ӵ�ǰ�ر߶���
                    frontQueue.push(edge);
                    activeFrontEdges[edge->id] = edge;
                }
            }
        }

        // ��ȡ��һ��Ҫ�����ǰ�ر�
        std::shared_ptr<Edge> getNextFrontEdge() {
            if (frontQueue.empty()) return nullptr;

            auto edge = frontQueue.top();
            frontQueue.pop();
            activeFrontEdges.erase(edge->id);

            return edge;
        }

        // ����µ�ǰ�ر�
        void addFrontEdge(const std::shared_ptr<Edge>& edge) {
            // ���±ߵ�״̬
            edge->updateStatus(angleThreshold);

            // ��ӵ�����
            frontQueue.push(edge);
            activeFrontEdges[edge->id] = edge;
        }

        // ����ǰ�ر�״̬
        void updateFrontEdgeStatus(const std::shared_ptr<Edge>& edge) {
            // ����߲��ڻ�Ծǰ�ر��б��У�ֱ�ӷ���
            if (activeFrontEdges.find(edge->id) == activeFrontEdges.end()) {
                return;
            }

            // ��¼�����ȼ�
            double oldPriority = edge->priority;

            // ����״̬
            edge->updateStatus(angleThreshold);

            // ������ȼ��ı䣬��Ҫ���²������
            if (edge->priority != oldPriority) {
                // �Ӷ������Ƴ���ʵ��ʵ������Ҫ����Ч�ķ�����
                std::vector<std::shared_ptr<Edge>> tempEdges;
                while (!frontQueue.empty()) {
                    auto e = frontQueue.top();
                    frontQueue.pop();
                    if (e->id != edge->id) {
                        tempEdges.push_back(e);
                    }
                }

                // �����б����²������
                for (auto& e : tempEdges) {
                    frontQueue.push(e);
                }

                // ������º�ı�
                frontQueue.push(edge);
            }
        }

        // �ж�ǰ�ر߶����Ƿ�Ϊ��
        bool isFrontQueueEmpty() const {
            return frontQueue.empty();
        }

        // ��ȡ��ǰǰ�ر�����
        size_t getFrontEdgeCount() const {
            return frontQueue.size();
        }

        // ����״̬����ĽǶ���ֵ
        void setAngleThreshold(double threshold) {
            angleThreshold = threshold;
        }
    };

} // namespace QMorph

#endif // FRONT_MANAGER_MODULE_HPP    