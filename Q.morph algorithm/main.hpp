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

    // Q-Morph算法主控制器
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
            // 初始化网格
            mesh = std::make_shared<Mesh>();

            // 初始化各模块
            triangulationModule = std::make_shared<TriangulationModule>(mesh);
            frontManagerModule = std::make_shared<FrontManagerModule>(mesh);
            quadGenerationModule = std::make_shared<QuadGenerationModule>(mesh, frontManagerModule);
            smoothingModule = std::make_shared<SmoothingModule>(mesh);
            topologyCleanupModule = std::make_shared<TopologyCleanupModule>(mesh);
            globalSmoothingOutputModule = std::make_shared<GlobalSmoothingOutputModule>(mesh);
        }

        // 设置三角化模块的尺寸函数
        void setSizeFunction(std::shared_ptr<SizeFunction> sizeFunction) {
            triangulationModule->setSizeFunction(sizeFunction);
        }

        // 执行Q-Morph算法
        void execute(const std::vector<Vector3D>& boundaryPoints,
            const std::vector<std::vector<int>>& boundaryLoops,
            const std::string& outputFilename,
            const std::string& outputFormat = "vtk") {
            // 1. 生成初始三角网格
            std::cout << "生成初始三角网格..." << std::endl;
            triangulationModule->generateTriangulation(boundaryPoints, boundaryLoops);

            // 2. 初始化前沿边队列
            std::cout << "初始化前沿边队列..." << std::endl;
            frontManagerModule->initializeFrontEdges();

            // 3. 生成四边形网格
            std::cout << "生成四边形网格..." << std::endl;
            int quadCount = 0;
            while (!frontManagerModule->isFrontQueueEmpty()) {
                auto frontEdge = frontManagerModule->getNextFrontEdge();
                if (quadGenerationModule->generateQuadFromFrontEdge(frontEdge)) {
                    ++quadCount;
                }

                // 定期平滑网格
                if (quadCount % 100 == 0) {
                    std::cout << "已生成 " << quadCount << " 个四边形，执行中间平滑..." << std::endl;
                    smoothingModule->smoothMesh();
                }
            }

            // 4. 最终平滑
            std::cout << "执行最终平滑..." << std::endl;
            smoothingModule->smoothMesh();

            // 5. 拓扑清理
            std::cout << "执行拓扑清理..." << std::endl;
            topologyCleanupModule->cleanupTopology();

            // 6. 全局平滑
            std::cout << "执行全局平滑..." << std::endl;
            globalSmoothingOutputModule->performGlobalSmoothing();

            // 7. 输出结果
            std::cout << "输出结果..." << std::endl;
            globalSmoothingOutputModule->outputMesh(outputFilename, outputFormat);
            globalSmoothingOutputModule->outputMeshQualityReport(outputFilename + ".quality.txt");

            std::cout << "Q-Morph算法执行完成!" << std::endl;
            std::cout << "共生成 " << mesh->quads.size() << " 个四边形" << std::endl;
        }
    };

} // namespace QMorph

#endif // QMORPH_CONTROLLER_HPP    