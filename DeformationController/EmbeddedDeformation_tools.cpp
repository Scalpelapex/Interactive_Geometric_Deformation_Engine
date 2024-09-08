/*
*   embedded deformation implementation
*   by Yanjie Xu
*   28/11/2023
*/

// dependencies
#include "EmbeddedDeformation_tools.h"
#include "downsampling.hpp"
#include "greedySearch.hpp"

//析构函数，基于输入点云文件创建
Deformation::Deformation(Eigen::MatrixXd V_in, Eigen::MatrixXi F_in, options opts) 
{
	std::cout << "use geodesic distance to look for closest point\n";

	// 保存顶点和面 in V_ and F_
	if (V_in.rows() == 3 && F_in.rows() == 3) {
		V_ = V_in.transpose();
		F_ = F_in.transpose();
		transpose_input_and_output_ = true;
	}
	else if (V_in.cols() == 3 && F_in.cols() == 3) {
		V_ = V_in;
		F_ = F_in;
		transpose_input_and_output_ = false;
	}
	else {
		throw std::invalid_argument("wrong input size");
	}
	// define nodes as subset (下采样计算nodes,但没有拓扑关系)
	Eigen::MatrixXd N;
	//downsampling(V_, N, indexes_of_deformation_graph_in_V_, opts.grid_resolution, use_farthest_sampling_);
	downsampling(V_, N, indexes_of_deformation_graph_in_V_,
		opts.grid_resolution, opts.grid_size,
		opts.use_farthest_sampling, opts.use_relative);

	// 设置选项
	verbose_ = true;
	use_knn_ = false;
	use_dijkstra_ = true;
	nodes_connectivity_ = opts.graph_connectivity;
	use_farthest_sampling_ = opts.use_farthest_sampling;
	
	// 初始化参数：
	w_rot_ = opts.w_rot;
	w_reg_ = opts.w_reg;
	w_rig_ = opts.w_rig;
	w_con_ = opts.w_con;

	V_sampled = N;
	k_nearest = 8	;
	sample_controls = 0;
	sample_nodes = N.rows();

	//rot置下采样点个数个单位阵，trans置下采样点个数个0
	InitRotAndTrans();
	//计算连接图，变量connectedMap类型maps<int,set<int>>保存着每个顶点id所连接的其他顶点set<int>
	CalConnectedMap();
	//计算下采样之后的顶点对应原模型中哪些顶点id，将结果储存在sample_idices中，类型未vector<int>
	CalSamplingVertices();
	//公式（4）- 计算权重变量，vector<vector<pair<int, float>>> embeddedWeights
	//其中embeddedWeights[i][j].first表示原始网格中第i个顶点的最近邻K个顶点中的第j个顶点的id是啥
	//embeddedWeights[i][j].second表示原始网格中第i个顶点的最近邻K个顶点中的第j个顶点的权重值
	CalEmbeddedWeights();
	//同上，但是计算的是采样网格(变形图)的顶点的对应权重
	CalDeformationGraphWeights();

	//Copy
	default_sampling_vertices = vector<Vector3d>(sample_nodes);
	for (int i = 0; i < sample_nodes; ++i)
	{
		default_sampling_vertices[i] = Vector3d(
			V_sampled(i, 0),
			V_sampled(i, 1),
			V_sampled(i, 2));
	}
	
	default_origin_vertices = vector<Vector3d>(V_.rows());
	for (int i = 0; i < V_.rows(); ++i)
	{
		default_origin_vertices[i] = Vector3d(V_(i, 0), V_(i, 1), V_(i, 2));
	}

	////Calculate the number of edges(仅用于流形的化简图)
	//for (auto iter = connectedMap.begin(); iter != connectedMap.end(); ++iter)
	//{
	//	sample_edges += (*iter).second.size();
	//}
	//sample_edges /= 2;
	sample_edges = deformation_graph_ptr_->num_edges();
	std::cout << "sample_edges = " << sample_edges << std::endl;
}

Deformation::Deformation(Eigen::MatrixXd V_in, Eigen::MatrixXi F_in, Eigen::MatrixXd VS_in, Eigen::MatrixXi FS_in, options opts)
{
	// extract point cloud
	nodes_connectivity_ = opts.graph_connectivity;
	verbose_ = opts.verbose;
	std::cout << "embedded deformation constructor: graph provided\n";
	std::cout << "embedded deformation constructor: graph provided\n";
	// check the data
	if (V_in.rows() == 3 && F_in.rows() == 3 && VS_in.rows() == 3 && FS_in.rows() == 3) {
		V_ = V_in.transpose();
		F_ = F_in.transpose();
		V_sampled = VS_in.transpose();
		F_sampled = FS_in.transpose();
		transpose_input_and_output_ = true;
	}
	else if (V_in.cols() == 3 && F_in.cols() == 3 && VS_in.cols() == 3 && FS_in.cols() == 3) {
		V_ = V_in;
		F_ = F_in;
		V_sampled = VS_in;
		F_sampled = FS_in;
		//deformation_graph_ptr_ = new libgraphcpp::Graph(VS_in, FS_in);
		transpose_input_and_output_ = false;
	}
	else {
		throw std::invalid_argument("wrong input size");
	}

	// 设置选项
	use_knn_ = true;
	use_dijkstra_ = false;
	nodes_connectivity_ = opts.graph_connectivity;
	verbose_ = opts.verbose;

	// 初始化参数：
	w_rot_ = opts.w_rot;
	w_reg_ = opts.w_reg;
	w_rig_ = opts.w_rig;
	w_con_ = opts.w_con;

	k_nearest = 8;
	sample_controls = 0;
	sample_nodes = V_sampled.rows();

	//rot置下采样点个数个单位阵，trans置下采样点个数个0
	InitRotAndTrans();
	//计算连接图，变量connectedMap类型maps<int,set<int>>保存着每个顶点id所连接的其他顶点set<int>
	CalConnectedMap_mesh();
	//计算下采样之后的顶点对应原模型中哪些顶点id，将结果储存在sample_idices中，类型未vector<int>
	CalSamplingVertices();
	//公式（4）- 计算权重变量，vector<vector<pair<int, float>>> embeddedWeights
	//其中embeddedWeights[i][j].first表示原始网格中第i个顶点的最近邻K个顶点中的第j个顶点的id是啥
	//embeddedWeights[i][j].second表示原始网格中第i个顶点的最近邻K个顶点中的第j个顶点的权重值
	CalEmbeddedWeights();
	//同上，但是计算的是采样网格(变形图)的顶点的对应权重
	CalDeformationGraphWeights();

	//Copy
	default_sampling_vertices = vector<Vector3d>(sample_nodes);
	for (int i = 0; i < sample_nodes; ++i)
	{
		default_sampling_vertices[i] = Vector3d(
			V_sampled(i, 0),
			V_sampled(i, 1),
			V_sampled(i, 2));
	}

	default_origin_vertices = vector<Vector3d>(V_.rows());
	for (int i = 0; i < V_.rows(); ++i)
	{
		default_origin_vertices[i] = Vector3d(V_(i, 0), V_(i, 1), V_(i, 2));
	}

	////Calculate the number of edges(仅用于流形的化简图)
	//for (auto iter = connectedMap.begin(); iter != connectedMap.end(); ++iter)
	//{
	//	sample_edges += (*iter).second.size();
	//}
	//sample_edges /= 2;
	sample_edges = deformation_graph_ptr_->num_edges();
	std::cout << "sample_edges = " << sample_edges << std::endl;
}

//  -----  Init -------
void Deformation::InitRotAndTrans()
{
	//rot是vector<Eigen::Matrix3f>类型，即一组3x3矩阵组成的vector
	//trans是<Eigen::Vector3f>类型，即一系列3X1的向量组成的vector
	rot.resize(sample_nodes);
	trans.resize(sample_nodes);
	Matrix3d temp_r = Matrix3d::Identity();
	Vector3d temp_t = Vector3d::Zero();
	//初始化rot 和 trans 类似于键值对，键-ID，值-对应的R和t
	for (int i = 0; i < sample_nodes; ++i)
	{
		rot[i] = temp_r;
		trans[i] = temp_t;
	}
	std::cout << "----Init: InitRotAndTrans Done."<<std::endl;
}

//该函数用于降采样之后有mesh的数据
//作用：计算连接图，connectedMap类型为vector<int,set<int>>
//第一个int表示下采样mesh的id，set<int>储存有该顶点所连接的所有顶点的id
void Deformation::CalConnectedMap_mesh()
{
	for (int i = 0; i < F_sampled.rows(); ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			// 在connectedMap中为当前顶点插入一个条目
			// 该条目是一个pair，第一个元素是顶点索引，
			// 第二个元素是一个空的 set<int> 用于存储连接的顶点。
			connectedMap.insert(pair<int, set<int>>(F_sampled(i, j), set<int>()));
			connectedMap[F_sampled(i, j)].insert(F_sampled(i, (j + 1)%3));
			connectedMap[F_sampled(i, j)].insert(F_sampled(i, (j + 2)%3));
		}
	}

	//Calculate the number of edges(仅用于流形的化简图)
	for (auto iter = connectedMap.begin(); iter != connectedMap.end(); ++iter)
	{
		sample_edges += (*iter).second.size();
	}
	sample_edges /= 2;
	MatrixXi E(sample_edges, 2);
	//MatrixXi E;
	vector<vector<int>> E_;
	int count = 0;
	for (int i = 0; i < V_sampled.rows(); ++i)
	{
		set<int> temp_list = connectedMap.at(i);
		for (auto it = temp_list.begin();it!=temp_list.end();++it)
		{
			if (*it < i) continue;
			E(count, 0) = i;
			E(count, 1) = *it;
			count++;
		}
	}
	deformation_graph_ptr_ = new libgraphcpp::Graph(V_sampled, E);
}

//该函数用于没有面的数据，只有下采样后的点云，通过贪心算法构建了graph
//这里初步计算了一个connectedmap
void Deformation::CalConnectedMap()
{
	// define edges???有问题
	Eigen::MatrixXi E(V_sampled.rows() * (nodes_connectivity_ + 1), 2);
	//indexes_of_deformation_graph_in_V_ 是一个vetor<int>
	//创建一个greedy_search变量，里面存着原mesh的V和F，以及下采样mesh的顶点坐标
	greedy_search search_object(V_, F_, indexes_of_deformation_graph_in_V_);
	std::vector<int> closest_points;
	int counter = 0;
	//遍历每个采样点：
	for (int i = 0; i < V_sampled.rows(); ++i) {
		closest_points = search_object.return_k_closest_points(indexes_of_deformation_graph_in_V_[i], nodes_connectivity_ + 2);
		closest_points.erase(closest_points.begin());
		set<int> temp_u_closet_points;
		for (int j = 0; j < closest_points.size(); j++) {
			temp_u_closet_points.insert(closest_points[j]);
			E(counter, 0) = i;
			E(counter, 1) = closest_points[j];
			counter++;
		}
		connectedMap.insert(pair<int, set<int>>(i, temp_u_closet_points));
	}
	// define deformation graph
	deformation_graph_ptr_ = new libgraphcpp::Graph(V_sampled, E);
	std::cout << "----Init: Created Graph Done." << endl;
}

void Deformation::CalSamplingVertices()
{
	//本算法已经有了，放在了变量indexes_of_deformation_graph_in_V_中
	//TO DO LIST:
	//1）得到采样网格的顶点对应于源网格顶点的编号：sample_idices
	//2) 将采样网格的顶点位置更新拟合到源网格顶点上；
}

void Deformation::CalEmbeddedWeights()
{
	int nodes = V_.rows();
	vector<vector<double>> distance(nodes);
	vector<vector<int>> index(nodes);
	//这里搜索最近的k个点，用的最原始的方法，两个for循环比较，速度会慢；
	//遍历原始mesh 的每个顶点
	for (int i = 0; i < nodes; i++)
	{
		vector<double> temp_d(sample_nodes);
		vector<int> temp_i(sample_nodes);
		vector3 temp_v;
		//遍历采样mesh 的每个顶点
		for (int j = 0; j < sample_nodes; j++)
		{
			temp_v = vector3(
				V_(i, 0) - V_sampled(j, 0),
				V_(i, 1) - V_sampled(j, 1),
				V_(i, 2) - V_sampled(j, 2)
			);
			/// 我是傻逼  +  号打成了乘号
			temp_d[j] = sqrt(temp_v[0] * temp_v[0] + temp_v[1] * temp_v[1] + temp_v[2] * temp_v[2]);
			temp_i[j] = j;
		}
		//以下两个变量保存着每个顶点对应采样网格每个顶点的id和对应的距离
		distance[i] = temp_d;
		index[i] = temp_i;
	}
	//储存原始网格中每个顶点与其最近的k个采样网格顶点的索引和距离
	embeddedWeights = vector<vector<pair<int, double>>>(nodes);
	for (int i = 0; i < nodes; ++i)
	{
		vector<pair<int, double>> temp_p(k_nearest + 1);
		//对于每个最近的采样网格顶点：
		for (int j = 0; j < k_nearest + 1; ++j)
		{
			int idx_min = j;

			for (int k = j + 1; k < sample_nodes; ++k)
			{
				if (distance[i][k] < distance[i][idx_min]) idx_min = k;
			}
			// 交换距离和索引，使得距离从小到大排序
			double temp = distance[i][j];
			distance[i][j] = distance[i][idx_min];
			distance[i][idx_min] = temp;

			int temp_i = index[i][j];
			index[i][j] = index[i][idx_min];
			index[i][idx_min] = temp_i;

			temp_p[j] = pair<int, double>(index[i][j], distance[i][j]);
		}
		// 存储每个原始网格顶点与其最近的 k 个采样网格顶点的索引和距离(包含距离为0的自己的点)
		embeddedWeights[i] = temp_p;
	}
	// 对每个原始网格顶点
	for (int i = 0; i < nodes; ++i)
	{
		// 计算[最近的k+1个采样网格顶点中]最远的采样网格顶点到当前原始网格顶点的距离
		vector3 temp = vector3(
			V_(i,0) - V_sampled(embeddedWeights[i][k_nearest].first, 0),
			V_(i,1) - V_sampled(embeddedWeights[i][k_nearest].first, 1),
			V_(i,2) - V_sampled(embeddedWeights[i][k_nearest].first, 2));
		double d_max = sqrt(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2]);

		// 初始化权重和权重之和
		double sum = 0;

		// 对于每个最近的 k 个采样网格顶点
		for (int j = 0; j < k_nearest; ++j)
		{
			temp = vector3(
				V_(i, 0) - V_sampled(embeddedWeights[i][j].first, 0),
				V_(i, 1) - V_sampled(embeddedWeights[i][j].first, 1),
				V_(i, 2) - V_sampled(embeddedWeights[i][j].first, 2));
			double d = sqrt(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2]);

			//权重值设置为原文公式（4）
			embeddedWeights[i][j].second = pow(1 - d / d_max, 2);
			sum += embeddedWeights[i][j].second;
		}

		//权重值归一化（原文公式4下面第一句话）
		for (int j = 0; j < k_nearest; ++j)
		{
			embeddedWeights[i][j].second /= sum;
			// 当 k_nearest 为1时，权重设为1
			if (k_nearest == 1) embeddedWeights[i][j].second = 1;
		}
	}
	std::cout << "----Init: Calculate EmbeddedWeights Done." << std::endl;
}

void Deformation::CalDeformationGraphWeights()
{
	vector<vector<double>> distance(sample_nodes);
	vector<vector<int>> index(sample_nodes);

	//这里的搜索最近点，用的最原始的方法（速度会慢）
	for (int i = 0; i < sample_nodes; ++i)
	{
		vector<double> temp_d(sample_nodes);
		vector<int> temp_i(sample_nodes);
		vector3 temp_v;
		for (int j = 0; j < sample_nodes; ++j)
		{
			temp_v = vector3(
				V_sampled(i, 0) - V_sampled(j, 0),
				V_sampled(i, 1) - V_sampled(j, 1),
				V_sampled(i, 2) - V_sampled(j, 2)
			);
			temp_d[j] = sqrt(temp_v[0] * temp_v[0] + temp_v[1] + temp_v[1] + temp_v[2] * temp_v[2]);
			temp_i[j] = j;
		}
		distance[i] = temp_d;
		index[i] = temp_i;
	}

	deformationGraphWeights = vector<vector<pair<int, double>>>(sample_nodes);
	for (int i = 0; i < sample_nodes; ++i)
	{
		vector<pair<int, double>> temp_p(k_nearest + 1);
		for (int j = 0; j < k_nearest + 1; ++j)
		{
			int idx_min = j;
			for (int k = j + 1; k < sample_nodes; ++k)
			{
				if (distance[i][k] < distance[i][idx_min]) idx_min = k;
			}
			double temp = distance[i][j];
			distance[i][j] = distance[i][idx_min];
			distance[i][idx_min] = temp;

			int temp_i = index[i][j];
			index[i][j] = index[i][idx_min];
			index[i][idx_min] = temp_i;

			temp_p[j] = pair<int, double>(index[i][j], distance[i][j]);
		}
		deformationGraphWeights[i] = temp_p;
	}

	for (int i = 0; i < sample_nodes; ++i)
	{
		vector3 temp = vector3(
			V_sampled(i, 0) - V_sampled(deformationGraphWeights[i][k_nearest].first, 0),
			V_sampled(i, 1) - V_sampled(deformationGraphWeights[i][k_nearest].first, 1),
			V_sampled(i, 2) - V_sampled(deformationGraphWeights[i][k_nearest].first, 2));
		double d_max = sqrt(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2]);
		double sum = 0;

		for (int j = 0; j < k_nearest; ++j)
		{
			temp = vector3(
				V_sampled(i, 0) - V_sampled(deformationGraphWeights[i][j].first, 0),
				V_sampled(i, 1) - V_sampled(deformationGraphWeights[i][j].first, 1),
				V_sampled(i, 2) - V_sampled(deformationGraphWeights[i][j].first, 2));
			double d = sqrt(temp[0] * temp[0] + temp[1] * temp[1] + temp[2] * temp[2]);

			deformationGraphWeights[i][j].second = pow(1 - d / d_max, 2);
			sum += deformationGraphWeights[i][j].second;
		}

		for (int j = 0; j < k_nearest; ++j)
		{
			deformationGraphWeights[i][j].second /= sum;
			if (k_nearest == 1) deformationGraphWeights[i][j].second = 1;
		}
	}
	std::cout << "----Init: Calculate DeformationGraphWeights Done." << std::endl;
}

//  ------- Calculate --------
void Deformation::GaussainNewton()
{
	const double epsilon = 1e-3;
	const int iter_max = 5;
	double err_current = 1, err_past = 2;
	SparseMatrix<double> J(6 * sample_nodes + 6 * sample_edges + 3 * sample_controls, 12 * sample_nodes);
	MatrixXd f, h, x(12 * sample_nodes, 1);

	InitRotAndTrans();
	//给x赋值为12个参数，即优化目标为R和t
	for (int i = 0; i < sample_nodes; ++i)
	{
		for (int j = 0; j < 9; ++j) x(i * 12 + j, 0) = rot[i](j / 3, j % 3);
		for (int j = 0; j < 3; ++j) x(i * 12 + 9 + j, 0) = trans[i](j);
	}
	std::cout << "---------------- Start ---------------" << std::endl;

	err_current = F(x);
	std::cout << "DEBUG-1 = " << err_current << std::endl;
	for (int i = 0; i<iter_max && fabs(err_past - err_current)>epsilon && err_current > epsilon; ++i)
	{
		err_past = err_current;
		Calf(f);
		CalJ(J);

		SparseMatrix<double> Jt = J.transpose();
		SparseMatrix<double> JtJ = J.transpose() * J;

		//这为啥写两遍
		//SimplicialCholesky<SparseMatrix<float>> solver(Jt * J);
		SimplicialCholesky<SparseMatrix<double>> solver(Jt * J);
		MatrixXd h = solver.solve(Jt * f);

		x = h;

		//std::ofstream outfile1("../x.txt"); 
		//outfile1 << x.format(Eigen::IOFormat(Eigen::StreamPrecision, 0," , ","\n","[","]","",""));
		//outfile1.close(); 

		cout << "---------------------------------------\n";
		cout << "Iteration: " << i + 1 << endl;
		err_current = F(x);
		cout << "Error: " << err_current << endl;
	}
	cout << "------------------- Finished -------------------\n" << endl;
}

void Deformation::Calf(MatrixXd& f)
{
	int idx = 0;
	MatrixXd fx = MatrixXd::Zero(6 * sample_nodes + (3 * sample_edges) * 2 + 3 * sample_controls, 1);

	// Erot
	for (int i = 0; i < sample_nodes; ++i)
	{
		fx(idx++, 0) = rot[i].col(0).dot(rot[i].col(1)) * sqrt(w_rot_);
		fx(idx++, 0) = rot[i].col(0).dot(rot[i].col(2)) * sqrt(w_rot_);
		fx(idx++, 0) = rot[i].col(1).dot(rot[i].col(2)) * sqrt(w_rot_);
		fx(idx++, 0) = sqrt(w_rot_);
		fx(idx++, 0) = sqrt(w_rot_);
		fx(idx++, 0) = sqrt(w_rot_);
	}

	// Ereg
	for (int j = 0; j < sample_nodes; ++j)
	{
		Vector3d gj = Vector3d(
			V_sampled(j, 0),
			V_sampled(j, 1),
			V_sampled(j, 2));
		for (auto iter = connectedMap[j].begin(); iter != connectedMap[j].end(); ++iter)
		{
			int k = *iter;
			Vector3d gk = Vector3d(
				V_sampled(k, 0),
				V_sampled(k, 1),
				V_sampled(k, 2));

			Vector3d temp = (gk - gj) * sqrt(w_reg_);

			fx(idx++, 0) = temp(0);
			fx(idx++, 0) = temp(1);
			fx(idx++, 0) = temp(2);
		}
	}

	// Econ
	// 需要知道控制点数据，control_points_data  :  vector<vector<vector3>>  vector3表示坐标, 两层；
	// 需要知道控制点id ， 
	for (int i = 0; i < control_points_data.size(); ++i)
	{
		for (int j = 0; j < control_points_data[i].size(); ++j)
		{
			Vector3d temp = Vector3d(
				control_points_data[i][j][0] - V_sampled(control_points_id[i][j], 0),
				control_points_data[i][j][1] - V_sampled(control_points_id[i][j], 1),
				control_points_data[i][j][2] - V_sampled(control_points_id[i][j], 2)) * sqrt(w_con_);

			fx(idx++, 0) = temp(0);
			fx(idx++, 0) = temp(1);
			fx(idx++, 0) = temp(2);
		}
	}

	f = fx;
}

void Deformation::CalJ(SparseMatrix<double>& J)
{
	int idx = 0;
	vector<Triplet<double>> Jacobi;

	// Erot
	for (int i = 0; i < sample_nodes; ++i)
	{
		Jacobi.push_back(Triplet<double>(idx, 0 + 12 * i, rot[i](0, 1)));
		Jacobi.push_back(Triplet<double>(idx, 1 + 12 * i, rot[i](1, 1)));
		Jacobi.push_back(Triplet<double>(idx, 2 + 12 * i, rot[i](2, 1)));

		Jacobi.push_back(Triplet<double>(idx, 3 + 12 * i, rot[i](0, 0)));
		Jacobi.push_back(Triplet<double>(idx, 4 + 12 * i, rot[i](1, 0)));
		Jacobi.push_back(Triplet<double>(idx, 5 + 12 * i, rot[i](2, 0)));

		idx++;

		Jacobi.push_back(Triplet<double>(idx, 0 + 12 * i, rot[i](0, 2)));
		Jacobi.push_back(Triplet<double>(idx, 1 + 12 * i, rot[i](1, 2)));
		Jacobi.push_back(Triplet<double>(idx, 2 + 12 * i, rot[i](2, 2)));

		Jacobi.push_back(Triplet<double>(idx, 6 + 12 * i, rot[i](0, 0)));
		Jacobi.push_back(Triplet<double>(idx, 7 + 12 * i, rot[i](1, 0)));
		Jacobi.push_back(Triplet<double>(idx, 8 + 12 * i, rot[i](2, 0)));

		idx++;

		Jacobi.push_back(Triplet<double>(idx, 3 + 12 * i, rot[i](0, 2)));
		Jacobi.push_back(Triplet<double>(idx, 4 + 12 * i, rot[i](1, 2)));
		Jacobi.push_back(Triplet<double>(idx, 5 + 12 * i, rot[i](2, 2)));

		Jacobi.push_back(Triplet<double>(idx, 6 + 12 * i, rot[i](0, 1)));
		Jacobi.push_back(Triplet<double>(idx, 7 + 12 * i, rot[i](1, 1)));
		Jacobi.push_back(Triplet<double>(idx, 8 + 12 * i, rot[i](2, 1)));

		idx++;

		for (int j = 0; j < 9; ++j)
		{
			if (j == 3 || j == 6) ++idx;
			Jacobi.push_back(Triplet<double>(idx, 12 * i + j, rot[i](j / 3, j % 3)));
		}
		idx++;
	}

	// Ereg
	for (int j = 0; j < sample_nodes; ++j)
	{
		for (auto iter = connectedMap[j].begin(); iter != connectedMap[j].end(); ++iter)
		{
			int k = *iter;

			vector3 ekj = vector3(
				V_sampled(k, 0) - V_sampled(j, 0),
				V_sampled(k, 1) - V_sampled(j, 1),
				V_sampled(k, 2) - V_sampled(j, 2)) * sqrt(w_reg_);

			for (int p = 0; p < 3; ++p)
			{
				for (int q = 0; q < 3; ++q)
				{
					Jacobi.push_back(Triplet<double>(idx, 12 * j + p * 3 + q, ekj[q]));
				}

				Jacobi.push_back(Triplet<double>(idx, 12 * j + 9 + p, sqrt(w_reg_)));
				Jacobi.push_back(Triplet<double>(idx, 12 * k + 9 + p, -sqrt(w_reg_)));

				idx++;
			}
		}
	}

	// Econ
	for (int i = 0; i < control_points_id.size(); ++i)
	{
		for (int j = 0; j < control_points_id[i].size(); ++j)
		{
			Jacobi.push_back(Triplet<double>(idx++, 9 + 12 * (control_points_id[i][j]) + 0, sqrt(w_con_)));
			Jacobi.push_back(Triplet<double>(idx++, 9 + 12 * (control_points_id[i][j]) + 1, sqrt(w_con_)));
			Jacobi.push_back(Triplet<double>(idx++, 9 + 12 * (control_points_id[i][j]) + 2, sqrt(w_con_)));
		}
	}

	J.setFromTriplets(Jacobi.begin(), Jacobi.end());
}

//公式(9)，计算偏差。
double Deformation::F(MatrixXd& x)
{
	double err = 0;

	for (int i = 0; i < sample_nodes; ++i)
	{
		for (int j = 0; j < 9; ++j) rot[i](j / 3, j % 3) = x(12 * i + j, 0);
		for (int j = 0; j < 3; ++j) trans[i][j] = x(12 * i + 9 + j, 0);
	}

	//更新原始mesh和变形图mesh
	UpdateOriginMesh();
	UpdateDeformationGraph();

	double Erot = CalErot(), Ereg = CalEreg(), Econ = CalEcon();

	cout << "Erot: " << Erot << ", Ereg: " << Ereg << ", Econ: " << Econ << endl;

	err = w_rot_ * Erot + w_reg_ * Ereg + w_con_ * Econ;

	return err;
}

//公式（7）计算E_reg
double Deformation::CalEreg()
{
	double Ereg = 0;

	for (int j = 0; j < sample_nodes; ++j)
	{
		Vector3d gj = Vector3d(
			V_sampled(j, 0),
			V_sampled(j, 1),
			V_sampled(j, 2));

		for (auto iter = connectedMap[j].begin(); iter != connectedMap[j].end(); ++iter)
		{
			int k = *iter;

			Vector3d gk = Vector3d(
				V_sampled(k, 0),
				V_sampled(k, 1),
				V_sampled(k, 2));

			//原式这里有个系数alpha_jk，文中设置为1.0，后续或许有用
			Ereg += pow((rot[j] * (gk - gj) + gj + trans[j] - (gk + trans[k])).norm(), 2);
		}
	}

	return Ereg;
}

//公式（5-6）计算E_rot
double Deformation::CalErot()
{
	double Erot = 0;

	for (int i = 0; i < sample_nodes; ++i)
	{
		Erot += pow(rot[i].col(0).dot(rot[i].col(1)), 2);
		Erot += pow(rot[i].col(0).dot(rot[i].col(2)), 2);
		Erot += pow(rot[i].col(1).dot(rot[i].col(2)), 2);
		Erot += pow(rot[i].col(0).dot(rot[i].col(0)) - 1, 2);
		Erot += pow(rot[i].col(1).dot(rot[i].col(1)) - 1, 2);
		Erot += pow(rot[i].col(2).dot(rot[i].col(2)) - 1, 2);
	}

	return Erot;
}

//公式（8）计算E_con
double Deformation::CalEcon()
{
	double Econ = 0;

	for (int i = 0; i < control_points_id.size(); ++i)
	{
		for (int j = 0; j < control_points_id[i].size(); ++j)
		{
			int idx = control_points_id[i][j];
			Econ += pow(fabs(V_sampled(idx, 0) - control_points_data[i][j][0]), 2);
			Econ += pow(fabs(V_sampled(idx, 1) - control_points_data[i][j][1]), 2);
			Econ += pow(fabs(V_sampled(idx, 2) - control_points_data[i][j][2]), 2);
		}
	}
	return Econ;
}

//  ------- Update -------
void Deformation::UpdateDeformationGraph()
{
	for (int i = 0; i < sample_nodes; ++i)
	{
		V_sampled(i, 0) += trans[i][0];
		V_sampled(i, 1) += trans[i][1];
		V_sampled(i, 2) += trans[i][2];
	}
}

void Deformation::UpdateSampleVertices()
{
	for (int i = 0; i < sample_nodes; ++i)
	{
		V_sampled(i, 0) = default_sampling_vertices[i][0];
		V_sampled(i, 1) = default_sampling_vertices[i][1];
		V_sampled(i, 2) = default_sampling_vertices[i][2];
	}

	for (int i = 0; i < V_.rows(); ++i)
	{
		V_(i, 0) = default_origin_vertices[i][0];
		V_(i, 1) = default_origin_vertices[i][1];
		V_(i, 2) = default_origin_vertices[i][2];
	}
}

//当rot和trans改变之后，更新原始网格坐标。
void Deformation::UpdateOriginMesh()
{
	int nodes = V_.rows();
	vector<Vector3d> results(nodes);
	for (int i = 0; i < nodes; ++i)
	{
		Vector3d result(0, 0, 0);
		Vector3d vi = Vector3d(
			V_(i, 0),
			V_(i, 1),
			V_(i, 2));

		/// <summary>
		/// 这里应该是有点问题：
		/// </summary>

		for (int j = 0; j < k_nearest; ++j)
		{
			Vector3d gj = Vector3d(
				V_sampled(embeddedWeights[i][j].first, 0),
				V_sampled(embeddedWeights[i][j].first, 1),
				V_sampled(embeddedWeights[i][j].first, 2));

			result += (embeddedWeights[i][j].second) * (rot[embeddedWeights[i][j].first] * (vi - gj) + gj + trans[embeddedWeights[i][j].first]);
		}
		results[i] = result;
	}

	for (int i = 0; i < nodes; ++i)
	{
		V_(i, 0) = results[i][0];
		V_(i, 1) = results[i][1];
		V_(i, 2) = results[i][2];
	}

	///  TEST
	std::ofstream outfile1("../V_TEST.txt");
	outfile1 << V_.format(Eigen::IOFormat(Eigen::StreamPrecision, 0, " , ", "\n", "[", "]", "", ""));
	outfile1.close();

}

//  -------- Receive Deform Data -------
void Deformation::SetControlPoints(Eigen::MatrixXd points)
{
	//vector<vector<int>> control_points_id;
	//vector<vector<vector3>> control_points_data;
	//vector<int> con_id1(points.rows());
	//vector<vector3> con_points1(points.rows());
	control_points_id.clear();
	control_points_data.clear();
	for (int i = 0;i<points.rows();++i)
	{
		int min_id = 9999;
		double min_d = 9999;
		double temp_d;
		vector3 temp_v;
		for (int j = 0; j < sample_nodes; ++j)
		{
			temp_v = vector3(
				points(i, 0) - V_sampled(j, 0), 
				points(i, 1) - V_sampled(j, 1),
				points(i, 2) - V_sampled(j, 2));
			temp_d = sqrt(temp_v[0] * temp_v[0] + temp_v[1] * temp_v[1] + temp_v[2] * temp_v[2]);
			if (temp_d < min_d)
			{
				min_d = temp_d;
				min_id = j;
			}
		}
		vector<int> temp_id;
		vector<vector3> temp_data;
		temp_id.push_back(min_id);
		temp_data.push_back(vector3(V_sampled(min_id, 0), V_sampled(min_id, 1), V_sampled(min_id, 2)));
		
		control_points_id.push_back(temp_id);
		control_points_data.push_back(temp_data);
		//con_id1[i] = min_id;
		//con_points1[i] = vector3(V_sampled(min_id, 0), V_sampled(min_id, 1), V_sampled(min_id, 2));
		
	}
	//control_points_id.push_back(con_id1);
	//control_points_data.push_back(con_points1);
	sample_controls = points.rows();
}

void Deformation::Deform(Eigen::MatrixXd old_points, Eigen::MatrixXd new_points, Eigen::MatrixXd &V_deform)
{
	Eigen::MatrixXd C_vec = new_points - old_points;
	for (int i = 0; i < control_points_data.size(); ++i)
	{
		for (int j = 0; j < control_points_data[i].size(); ++j)
		{
			int idx = control_points_id[i][j];
			control_points_data[i][j] += vector3(C_vec(i, 0), C_vec(i, 1), C_vec(i, 2));
		}
	}
	Run();
	V_deform = V_;
	//V_deform = V_.transpose();
}
//  ------- Show -------
void Deformation::show_deformation_graph()
{
	visualization::plot(*deformation_graph_ptr_, "deformation graph");
}

void Deformation::show_param()
{
	cout << "--------------------------" << endl;
	cout << "Origin_nodes:" << V_.rows() << endl;
	
	cout << "sample_nodes:" << sample_nodes << endl;
	cout << "sample_edges:" << sample_edges << endl;
	cout << "sample_controls:" << sample_controls << endl;
	cout << "k_nearest:" << k_nearest << endl;
	cout << "--------------------------" << endl;
}

//  -------- Run --------
void Deformation::Run()
{
	UpdateSampleVertices();
	GaussainNewton();
}