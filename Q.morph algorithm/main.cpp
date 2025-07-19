#include "mesh_structure.hpp"
#include <iostream>
#include <vector>
#include "main.hpp"

int main() {
    try {
        // ����Q-Morph������
        QMorph::QMorphController controller;

        // ���óߴ纯��������ʹ�þ��ȳߴ磩
        auto sizeFunction = std::make_shared<QMorph::UniformSizeFunction>(0.1);
        controller.setSizeFunction(sizeFunction);

        // ����߽�㣨����ʹ��һ���򵥵ľ��α߽���Ϊʾ����
        std::vector<QMorph::Vector3D> boundaryPoints = {
            QMorph::Vector3D(0, 0, 0),
            QMorph::Vector3D(1, 0, 0),
            QMorph::Vector3D(1, 1, 0),
            QMorph::Vector3D(0, 1, 0)
        };

        // ����߽绷��һ���պϵı߽磩
        std::vector<std::vector<int>> boundaryLoops = {
            {0, 1, 2, 3} // ���ε��ĸ���������
        };

        // ִ��Q-Morph�㷨
        controller.execute(boundaryPoints, boundaryLoops, "output.vtk");

        return 0;
    }
    catch (const std::exception& e) {
        std::cerr << "����: " << e.what() << std::endl;
        return 1;
    }
}