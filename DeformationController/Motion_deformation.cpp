#include <yaml-cpp/yaml.h>
#include "options.hpp"
//#include "embedded_deformation/embedDeform.hpp"
#include "EmbeddedDeformation_tools.h"
#include "polyscope/polyscope.h"
#include "utils/IO/readPLY.h"
#include "utils/IO/readCSV.h"
#include "utils/IO/writePLY.h"
#include "utils/visualization/plotMesh.h"
#include "utils/visualization/plotCloud.h"

#include "Serial.h"
#include <TCHAR.h>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <iostream>
#include "polyscope/surface_mesh.h"

Eigen::MatrixXd V;
Eigen::MatrixXi F;

polyscope::SurfaceMesh* mesh;
Deformation* non_rigid_deformation;
CSerial serial;
Eigen::MatrixXd new_points, old_points;
/*double points[18] = {0.0,  1.0,  0.0,
                     0.0, -1.0,  0.0,
                     1.0,  0.0,  0.0,
                    -1.0,  0.0,  0.0,
                     0.0,  0.0,  1.0,
                     0.0,  0.0, -1.0 };*/
double points[18] = { -81.0,  -97.0,  -166.0,
                     -46.760380, -115.265938, -129.528473,
                     -79.170967, - 115.810219, - 208.022659,
                    -72.478783, - 142.090134, - 162.838928,
                     -77.299461, - 116.004593, - 169.209656,
                     -68.007629, - 110.468216, - 137.559525, };
char buf[512];
void myCallback()
{
    /*DWORD dwRead;
    BOOL bReadStat = ReadFile(serial.m_hComm,
        buf,
        sizeof(buf) - 1,
        &dwRead,
        NULL);
    if (bReadStat && dwRead > 0) {
        buf[dwRead] = '\0';
        cout << buf[1] << endl;;
    }*/
    /*if (test(serial, buf))
    {
        cout << buf[2] << endl;
    }*/
    //cout << "Get Data:" << serial.ReceiveData() << endl;;
    if (Receive_Deformation2(serial, buf, new_points, old_points, points))
    {
        cout << "New_points:\n" << new_points << endl;
        cout << "Old_points:\n" << old_points << endl;

        non_rigid_deformation->SetControlPoints(old_points);
        Eigen::MatrixXd V_deformed;
        non_rigid_deformation->Deform(old_points, new_points, V_deformed);
        mesh->updateVertexPositions(V_deformed);
    }
    PurgeComm(serial.m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
}

int main()
{
    // 参数初始化
    options opts;
    //char tmp[256];
    //GetModuleFileNameA(NULL, tmp, MAX_PATH);
    //printf("paht = %s\n", tmp);
    //注意：修改下面配置文件的路径和配置文件内模型的路径
    //Note: You need to modify this path and the content of the config-file in this path.
    opts.loadYAML("../../../../config/config.yaml");
    std::cout << "** Progress: yaml loaded\n";

    // 初始化可视化框架窗口Polyscope C++
    polyscope::init();
    std::cout << "** Progress: plyscope initialized\n";

    // 串口初始化：
    serial.OpenSerialPort(_T("COM6:"), 9600, 8, 1);
    std::cout << "Open Uart - COM6: Sucessfully." << std::endl;

    // 模型初始化
    Eigen::MatrixXd _V_pre,_V, VS;
    Eigen::MatrixXi _F, FS;
    std::cout << "** Progress: load file ...";
    readPLY(opts.path_input_file, V, F);
    //V = _V.transpose();
    //F = _F.transpose();
    // check for error
    if (opts.use_geodesic && F.rows() == 0)
    {
        std::cout << "Config file error: use_geodesic = true, but nor faces were provided." << std::endl;
        exit(-1);
    }
    if (opts.graph_provided)
    {
        readPLY(opts.path_graph_obj, VS, FS);
        non_rigid_deformation = new Deformation(V, F, VS, FS, opts);
    }
    else
    {
        if (opts.use_geodesic)
            non_rigid_deformation = new Deformation(V, F, opts);
    }
    std::cout << "** Progress: Create Embedded Deformation Done.\n";
    //将模型注册到框架中：
    // 修正
    _V = V.transpose();
    V = V.reshaped(V.cols(),3);
    V = _V;
    _F = F.transpose();
    F = F.reshaped(F.cols(), 3);
    F = _F;

    mesh = polyscope::registerSurfaceMesh("Ball", V, F);

    std::cout << "Hello Polyscope!" << std::endl;

    polyscope::state::userCallback = myCallback;

    //non_rigid_deformation->show_deformation_graph();
    /*while (1) {
        if (Receive_Deformation(serial, buf, new_points, old_points))
        {
            cout << "New_points:\n" << new_points << endl;
            cout << "Old_points:\n" << old_points << endl;

            non_rigid_deformation->SetControlPoints(old_points);
            Eigen::MatrixXd V_deformed;
            non_rigid_deformation->Deform(old_points, new_points, V_deformed);
            mesh->updateVertexPositions(V_deformed);
            polyscope::show();
        }
        PurgeComm(serial.m_hComm, PURGE_RXCLEAR | PURGE_TXCLEAR);
    }*/

    polyscope::show();

}