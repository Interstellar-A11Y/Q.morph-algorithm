#ifndef GLOBAL_SMOOTHING_OUTPUT_MODULE_HPP
#define GLOBAL_SMOOTHING_OUTPUT_MODULE_HPP

#include "mesh_structure.hpp"
#include <memory>
#include <vector>
#include <string>

namespace QMorph {

    // ȫ��ƽ���������ģ��
    class GlobalSmoothingOutputModule {
    private:
        std::shared_ptr<Mesh> mesh;
        int globalSmoothingIterations; // ȫ��ƽ����������

    public:
        GlobalSmoothingOutputModule(std::shared_ptr<Mesh> mesh)
            : mesh(mesh), globalSmoothingIterations(3) {}

        // ����ȫ��ƽ����������
        void setGlobalSmoothingIterations(int iterations) {
            globalSmoothingIterations = iterations;
        }

        // ִ��ȫ��ƽ��
        void performGlobalSmoothing() {
            for (int i = 0; i < globalSmoothingIterations; ++i) {
                smoothMeshOnce();
            }
        }

        // ִ��һ��ȫ��ƽ��
        void smoothMeshOnce() {
            // ���Ƶ�ǰ�ڵ�λ��
            std::vector<Vector3D> oldPositions(mesh->nodes.size());
            for (size_t i = 0; i < mesh->nodes.size(); ++i) {
                oldPositions[i] = mesh->nodes[i]->coordinates;
            }

            // �������нڵ㣬������λ��
            for (auto& node : mesh->nodes) {
                // �����߽�ڵ�
                if (node->isBoundary) continue;

                // ������λ��
                Vector3D newPosition = calculateConstrainedLaplacianPosition(node);

                // ֻ�е���λ���ܸ�����״����ʱ���ƶ�
                if (wouldPositionImproveQuality(node, newPosition)) {
                    node->coordinates = newPosition;
                }
            }
        }

        // ����Լ��������˹λ��
        Vector3D calculateConstrainedLaplacianPosition(const std::shared_ptr<Node>& node) {
            // ��ʵ�֣�ʵ����������ڽڵ�ļ�Ȩƽ��λ��
            return node->coordinates;
        }

        // �ж���λ���Ƿ��ܸ�����״����
        bool wouldPositionImproveQuality(const std::shared_ptr<Node>& node,
            const Vector3D& newPosition) {
            // ��ʵ�֣�ʵ����Ƚ��¾�λ�õ�����ָ��
            return true;
        }

        // ���������
        void outputMesh(const std::string& filename, const std::string& format = "vtk") {
            if (format == "vtk") {
                mesh->exportToVTK(filename);
            }
            else if (format == "stl") {
                // ʵ��STL���
            }
            else if (format == "inp") {
                // ʵ��INP���
            }
            else {
                throw std::runtime_error("��֧�ֵ������ʽ: " + format);
            }
        }

        // ���������������
        void outputMeshQualityReport(const std::string& filename) {
            std::ofstream file(filename);
            if (!file.is_open()) {
                throw std::runtime_error("�޷����ļ�: " + filename);
            }

            auto quality = mesh->calculateMeshQuality();

            file << "������������\n";
            file << "------------------------\n";
            file << "�ı�������: " << mesh->quads.size() << "\n";
            file << "������ڵ�����: " << quality.irregularNodeCount << "\n";
            file << "ƽ������ָ��(��): " << quality.avgQuality << "\n";
            file << "��С����ָ��(��): " << quality.minQuality << "\n";
            file << "�������ָ��(��): " << quality.maxQuality << "\n";

            file.close();
        }
    };

} // namespace QMorph

#endif // GLOBAL_SMOOTHING_OUTPUT_MODULE_HPP    