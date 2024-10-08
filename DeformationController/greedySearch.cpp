#include <vector>
#include <limits> 			// for numeric_limits
#include <iostream>
 
#include "greedySearch.hpp"

const weight_t max_weight = std::numeric_limits<double>::infinity();


double distance_between_points(Eigen::Vector3d a, Eigen::Vector3d b)
{
	return sqrt( pow(a(0) - b(0), 2) + pow(a(1) - b(1), 2) + pow(a(2) - b(2), 2));
}

bool is_element_in_vector(int a, std::vector<int> & A, int & element_position)
{
	bool point_is_in_set = false;

	for (int i = 0; i < A.size(); ++i)
		if ( a == A.at(i) )
		{
			element_position = i;
			point_is_in_set = true;
		}

	return point_is_in_set;
}


greedy_search::greedy_search(Eigen::MatrixXd V, 
							 Eigen::MatrixXi F,
							 std::vector<int> indexes_of_sub_V_in_V)
{
	// set private variables
	V_ = V;
	F_ = F;
	//sub_V_ = sub_V;
	graph_size_ = V_.rows();

	// circulate once through each face
	//类型为vector<vector<neighbor>>
	//定义最外层有原mesh顶点个数个，但每个vector元素上的vector内有几个neighbor不知道；
	adjacency_list_.resize(V.rows());
	for (int i = 0; i < F.rows(); ++i)
	{
		adjacency_list_.at( F(i, 0) ).push_back(neighbor(F(i,1), distance_between_points(V.row(F(i,0)), V.row(F(i,1)) ) ));
		adjacency_list_.at( F(i, 1) ).push_back(neighbor(F(i,2), distance_between_points(V.row(F(i,1)), V.row(F(i,2)) ) ));
		adjacency_list_.at( F(i, 2) ).push_back(neighbor(F(i,0), distance_between_points(V.row(F(i,2)), V.row(F(i,0)) ) ));
	}

	// indexes_of_sub_V_in_V_ could be set as an input instead of sub_V
	indexes_of_sub_V_in_V_ = indexes_of_sub_V_in_V;
}


std::vector< int > greedy_search::return_k_closest_points(int source, int k)
{
	std::vector< int > output;	

	// initialize the distances to all nodes:
	std::vector<double> min_distance;
	// 大小：V_rows() ,无穷
	min_distance.resize(graph_size_, max_weight);

	min_distance.at(source) = 0;

	// set the set of visited nodes:
	std::vector<int> visited;
	std::vector<int> to_visit;

	// initialize the node to start from
	int u = source;
	int element_position;

	// start searching
	int sub_V_found = 0;
	//循环k个点，如果找到了k个点，就结束  输入的k = node_connectivity + 2
	while (sub_V_found != k)
	{
		//u代表当前下采样网格中的某个点u
		//遍历原网格u点的所有连接点
		for (int i = 0; i < adjacency_list_.at(u).size(); ++i)
		{

			//判断u点的第i个连接点是否在to_visit或者visit中，现在to_visit中搜索，再在visit中搜索，都没有就将该点放在to_visit中
			if (! is_element_in_vector(adjacency_list_.at(u).at(i).target, to_visit, element_position))
				if (! is_element_in_vector(adjacency_list_.at(u).at(i).target, visited, element_position))
					to_visit.push_back(adjacency_list_.at(u).at(i).target);

			//更新点min_distance
			//如果u点的min_distance + u和i之间的距离 < i点的min_distance 
			if ( min_distance.at(u) +  adjacency_list_.at(u).at(i).weight < min_distance.at( adjacency_list_.at(u).at(i).target ) )
			{
				//u点第i个连接点的min_distance = u点的min_distance（0） + u点和i点之间的距离
				min_distance.at( adjacency_list_.at(u).at(i).target ) = min_distance.at(u) +  adjacency_list_.at(u).at(i).weight;
			}
		}
		//访问完了就将u放在visited中，贪心算法的特点。
		visited.push_back(u);

		// check if the visited point is in sub_V
		if ( is_element_in_vector(u, indexes_of_sub_V_in_V_, element_position) )
		{
			++ sub_V_found;
			output.push_back(element_position);
		}

		// set next u
		int index_of_next_point = 0;
		for (int i = 0; i < to_visit.size(); ++i)
			if (min_distance.at( to_visit.at(i) ) < min_distance.at( to_visit.at( index_of_next_point) ) )
				index_of_next_point = i;
				
		u = to_visit.at(index_of_next_point);
		to_visit.erase(to_visit.begin() + index_of_next_point);
	}

	return output;
}
