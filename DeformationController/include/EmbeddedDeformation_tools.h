/*
*   embedded deformation implementation
*   by Yanjie. Xu
*   28/11/2023
*/
#ifndef EMBEDDEDDEFORMATION_TOOLS
#define EMBEDDEDDEFORMATION_TOOLS
//F版本头文件
#include "options.hpp"
#include "libGraphCpp/graph.hpp"
#include "libGraphCpp/plotGraph.hpp"

//P版本头文件
#include "mtxlib.h"
#include <vector>
#include <set>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <Eigen/SparseCholesky>

using namespace std;
using namespace Eigen;

class Deformation
{
public:
	// 默认输入的点云文件带有顶点和三角面数据
	Deformation(Eigen::MatrixXd V_in,Eigen::MatrixXi F_in,options opts);

	// 测试：变形图采用均匀的：
	Deformation(Eigen::MatrixXd V_in, Eigen::MatrixXi F_in, Eigen::MatrixXd N_in, Eigen::MatrixXi E_in, options opts);


	// destructor
	~Deformation() {
		if (deformation_graph_ptr_ != nullptr) delete deformation_graph_ptr_;
	}
	void Run();
	void SetControlPoints(Eigen::MatrixXd points);
	//void SetControlPointsTranslate(int selectedId, vector3 vec);
	void Deform(Eigen::MatrixXd old_points, Eigen::MatrixXd new_points, Eigen::MatrixXd& V_deform);
	void show_deformation_graph();
	void show_param();

private:
	void InitRotAndTrans();
	void CalConnectedMap();
	void CalConnectedMap_mesh();
	void CalSamplingVertices();
	void CalDeformationGraphWeights();
	void CalEmbeddedWeights();

private:
	void GaussainNewton();
	void Calf(MatrixXd& f);
	void CalJ(SparseMatrix<double>& J);
	double F(MatrixXd& x);
	double CalErot();
	double CalEcon();
	double CalEreg();
	void UpdateSampleVertices();
	void UpdateDeformationGraph();
	void UpdateOriginMesh();


private:
	Eigen::MatrixXd V_;
	Eigen::MatrixXi F_;
	Eigen::MatrixXd V_sampled;
	Eigen::MatrixXi F_sampled;
	libgraphcpp::Graph* deformation_graph_ptr_ = nullptr;

private:
	// options
	bool use_knn_;
	bool use_dijkstra_;
	bool use_farthest_sampling_ = false;
	bool verbose_;
	int nodes_connectivity_;
	bool transpose_input_and_output_;

private:
	double w_rot_ = 1;
	double w_reg_ = 10;
	double w_rig_ = 10;
	double w_con_ = 100;

private:
	int sample_nodes = 0;
	int sample_edges = 0;
	int sample_controls = 0;
	int k_nearest = 0;
	vector<int> indexes_of_deformation_graph_in_V_;
	vector<Matrix3d> rot;
	vector<Vector3d> trans;
	vector<Vector3d> default_sampling_vertices;
	vector<Vector3d> default_origin_vertices;
	// - 多个控制块（未测试）
	vector<vector<int>> control_points_id;
	vector<vector<vector3>> control_points_data;
	// - 单个控制序列 （正在测试）
	//vector<int> control_points_id;
	//vector<vector3> control_points_data;
	vector<vector<pair<int, double>>> deformationGraphWeights;
	vector<vector<pair<int, double>>> embeddedWeights;
	map<int, set<int>> connectedMap;
};

#endif // !1
