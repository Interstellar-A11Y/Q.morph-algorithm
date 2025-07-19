#ifndef FRONT_MANAGER_MODULE_HPP
#define FRONT_MANAGER_MODULE_HPP

#include "mesh_structure.hpp"
#include <memory>
#include <vector>
#include <queue>
#include <map>
#include <functional>

namespace QMorph {

    // 前沿边比较器，用于优先队列
    struct FrontEdgeComparator {
        bool operator()(const std::shared_ptr<Edge>& e1, const std::shared_ptr<Edge>& e2) const {
            return e1->priority < e2->priority;
        }
    };

    // 前沿边管理模块
    class FrontManagerModule {
    private:
        std::shared_ptr<Mesh> mesh;
        std::priority_queue<std::shared_ptr<Edge>, std::vector<std::shared_ptr<Edge>>, FrontEdgeComparator> frontQueue;
        std::map<int, std::shared_ptr<Edge>> activeFrontEdges; // ID到前沿边的映射
        double angleThreshold; // 状态分类的角度阈值

    public:
        FrontManagerModule(std::shared_ptr<Mesh> mesh)
            : mesh(mesh), angleThreshold(2.35619) {} // 3π/4

        // 初始化前沿边队列
        void initializeFrontEdges() {
            // 清空现有队列
            while (!frontQueue.empty()) {
                frontQueue.pop();
            }
            activeFrontEdges.clear();

            // 遍历所有边，找出边界边（仅相邻1个三角形的边）
            for (auto& edge : mesh->edges) {
                if (edge->triangles.size() == 1) {
                    // 更新边的状态
                    edge->updateStatus(angleThreshold);

                    // 添加到前沿边队列
                    frontQueue.push(edge);
                    activeFrontEdges[edge->id] = edge;
                }
            }
        }

        // 获取下一个要处理的前沿边
        std::shared_ptr<Edge> getNextFrontEdge() {
            if (frontQueue.empty()) return nullptr;

            auto edge = frontQueue.top();
            frontQueue.pop();
            activeFrontEdges.erase(edge->id);

            return edge;
        }

        // 添加新的前沿边
        void addFrontEdge(const std::shared_ptr<Edge>& edge) {
            // 更新边的状态
            edge->updateStatus(angleThreshold);

            // 添加到队列
            frontQueue.push(edge);
            activeFrontEdges[edge->id] = edge;
        }

        // 更新前沿边状态
        void updateFrontEdgeStatus(const std::shared_ptr<Edge>& edge) {
            // 如果边不在活跃前沿边列表中，直接返回
            if (activeFrontEdges.find(edge->id) == activeFrontEdges.end()) {
                return;
            }

            // 记录旧优先级
            double oldPriority = edge->priority;

            // 更新状态
            edge->updateStatus(angleThreshold);

            // 如果优先级改变，需要重新插入队列
            if (edge->priority != oldPriority) {
                // 从队列中移除（实际实现中需要更高效的方法）
                std::vector<std::shared_ptr<Edge>> tempEdges;
                while (!frontQueue.empty()) {
                    auto e = frontQueue.top();
                    frontQueue.pop();
                    if (e->id != edge->id) {
                        tempEdges.push_back(e);
                    }
                }

                // 将所有边重新插入队列
                for (auto& e : tempEdges) {
                    frontQueue.push(e);
                }

                // 插入更新后的边
                frontQueue.push(edge);
            }
        }

        // 判断前沿边队列是否为空
        bool isFrontQueueEmpty() const {
            return frontQueue.empty();
        }

        // 获取当前前沿边数量
        size_t getFrontEdgeCount() const {
            return frontQueue.size();
        }

        // 设置状态分类的角度阈值
        void setAngleThreshold(double threshold) {
            angleThreshold = threshold;
        }
    };

} // namespace QMorph

#endif // FRONT_MANAGER_MODULE_HPP    