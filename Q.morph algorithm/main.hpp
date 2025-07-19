#ifndef QMORPH_CONTROLLER_HPP
#define QMORPH_CONTROLLER_HPP

#include "triangulation_module.hpp"
#include "front_manager_module.hpp"
#include "quad_generation_module.hpp"
#include "smoothing_module.hpp"
#include "topology_cleanup_module.hpp"
#include "global_smoothing_output_module.hpp"
#include <memory>
#include <iostream>

namespace QMorph {

    // Q-Morph�㷨��������
    class QMorphController {
    private:
        std::shared_ptr<Mesh> mesh;
        std::shared_ptr<TriangulationModule> triangulationModule;
        std::shared_ptr<FrontManagerModule> frontManagerModule;
        std::shared_ptr<QuadGenerationModule> quadGenerationModule;
        std::shared_ptr<SmoothingModule> smoothingModule;
        std::shared_ptr<TopologyCleanupModule> topologyCleanupModule;
        std::shared_ptr<GlobalSmoothingOutputModule> globalSmoothingOutputModule;

    public:
        QMorphController() {
            // ��ʼ������
            mesh = std::make_shared<Mesh>();

            // ��ʼ����ģ��
            triangulationModule = std::make_shared<TriangulationModule>(mesh);
            frontManagerModule = std::make_shared<FrontManagerModule>(mesh);
            quadGenerationModule = std::make_shared<QuadGenerationModule>(mesh, frontManagerModule);
            smoothingModule = std::make_shared<SmoothingModule>(mesh);
            topologyCleanupModule = std::make_shared<TopologyCleanupModule>(mesh);
            globalSmoothingOutputModule = std::make_shared<GlobalSmoothingOutputModule>(mesh);
        }

        // �������ǻ�ģ��ĳߴ纯��
        void setSizeFunction(std::shared_ptr<SizeFunction> sizeFunction) {
            triangulationModule->setSizeFunction(sizeFunction);
        }

        // ִ��Q-Morph�㷨
        void execute(const std::vector<Vector3D>& boundaryPoints,
            const std::vector<std::vector<int>>& boundaryLoops,
            const std::string& outputFilename,
            const std::string& outputFormat = "vtk") {
            // 1. ���ɳ�ʼ��������
            std::cout << "���ɳ�ʼ��������..." << std::endl;
            triangulationModule->generateTriangulation(boundaryPoints, boundaryLoops);

            // 2. ��ʼ��ǰ�ر߶���
            std::cout << "��ʼ��ǰ�ر߶���..." << std::endl;
            frontManagerModule->initializeFrontEdges();

            // 3. �����ı�������
            std::cout << "�����ı�������..." << std::endl;
            int quadCount = 0;
            while (!frontManagerModule->isFrontQueueEmpty()) {
                auto frontEdge = frontManagerModule->getNextFrontEdge();
                if (quadGenerationModule->generateQuadFromFrontEdge(frontEdge)) {
                    ++quadCount;
                }

                // ����ƽ������
                if (quadCount % 100 == 0) {
                    std::cout << "������ " << quadCount << " ���ı��Σ�ִ���м�ƽ��..." << std::endl;
                    smoothingModule->smoothMesh();
                }
            }

            // 4. ����ƽ��
            std::cout << "ִ������ƽ��..." << std::endl;
            smoothingModule->smoothMesh();

            // 5. ��������
            std::cout << "ִ����������..." << std::endl;
            topologyCleanupModule->cleanupTopology();

            // 6. ȫ��ƽ��
            std::cout << "ִ��ȫ��ƽ��..." << std::endl;
            globalSmoothingOutputModule->performGlobalSmoothing();

            // 7. ������
            std::cout << "������..." << std::endl;
            globalSmoothingOutputModule->outputMesh(outputFilename, outputFormat);
            globalSmoothingOutputModule->outputMeshQualityReport(outputFilename + ".quality.txt");

            std::cout << "Q-Morph�㷨ִ�����!" << std::endl;
            std::cout << "������ " << mesh->quads.size() << " ���ı���" << std::endl;
        }
    };

} // namespace QMorph

#endif // QMORPH_CONTROLLER_HPP    