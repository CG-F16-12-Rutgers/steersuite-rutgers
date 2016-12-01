//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"

#include <unordered_map>
#include <chrono>

namespace SteerLib
{

	// A4 Heap
	class My_heap
        {
        public:

                void clear()
                {
                        f_heap.clear();
                        g_heap.clear();
                        h_heap.clear();
                        index_heap.clear();
                        index_to_node.clear();
                }

                void init()
                {
                        f_heap.push_back(-1);
                        g_heap.push_back(-1);
                        h_heap.push_back(-1);
                        index_heap.push_back(-1);
                }

                My_heap()
                {
                        clear();
                        init();
                }

                ~My_heap()
                {
                        clear();
                }

		// return true when a is better than b
                static bool better(double fa, double ga, double fb, double gb)
                {
                        if (fa < fb)
                                return true;
                        if (fa > fb)
                                return false;
                        return (ga < gb);
                }

                // move the node to proper position so that the heap is maintained, return new position
                int heapify(int node)
                {
                        if (node >= index_heap.size())
                                return -1;
                        int res = node, n = f_heap.size();
                        // go up
                        while (res > 1 && better(f_heap[res], g_heap[res], f_heap[res >> 1], g_heap[res >> 1]))
                        {
                                swap(res, (res >> 1));
                                res = (res >> 1);
                        }
                        // go down
                        while ((res << 1) < n)
                        {
                                int left = (res << 1), right = 1 + (res << 1), best_child;
                                if (right >= n || better(f_heap[left], g_heap[left], f_heap[right], g_heap[right]))
                                        best_child = left;
                                else
                                        best_child = right;
                                if (better(f_heap[res], g_heap[res], f_heap[best_child], g_heap[best_child]))
                                        break;
                                swap(res, best_child);
                                res = best_child;
                        }
                        return res;
                }

                // return true if the heap is empty
                bool empty()
                {
                        return (f_heap.size() < 2);
                }

		bool insert(int index, double f_val, double g_val, double h_val)
                {
                        f_heap.push_back(f_val);
                        g_heap.push_back(g_val);
                        h_heap.push_back(h_val);
                        index_heap.push_back(index);
                        index_to_node[index] = index_heap.size() - 1;
                        heapify(index_to_node[index]);
                        return true;
                }

                // return true when cell with index exists in the heap
                bool has_index(int index)
                {
                        if (index_to_node.find(index) == index_to_node.end())
                                return false;
                        return true;
                }

                bool get_root(int &index, double &f_val, double &g_val, double &h_val)
                {
                        if (empty())
                                return false;
                        index = index_heap[1];
                        f_val = f_heap[1];
                        g_val = g_heap[1];
                        h_val = h_heap[1];
                        return true;
                }

                bool pop()
                {
                        if (empty())
                                return false;
                        int n = f_heap.size() - 1, removed_index;
                        swap(1, n);
                        removed_index = index_heap[n];
                        f_heap.pop_back();
                        g_heap.pop_back();
                        h_heap.pop_back();
                        index_heap.pop_back();
                        index_to_node.erase(removed_index);
                        heapify(1);
                        return true;
                }

                bool update(int index, double f_val, double g_val, double h_val)
                {
                        if (!has_index(index))
                                return false;
                        int node = index_to_node[index];
                        f_heap[node] = f_val;
                        g_heap[node] = g_val;
                        h_heap[node] = h_val;
                        heapify(node);
                        return true;
                }

                bool remove_cell_by_index(int index)
                {
                        if (!has_index(index))
                                return false;
                        int node = index_to_node[index], n = index_heap.size() - 1;
                        swap(node, n);
                        f_heap.pop_back();
                        g_heap.pop_back();
                        h_heap.pop_back();
                        index_heap.pop_back();
                        index_to_node.erase(index);
                        heapify(node);
                        return true;
                }

        private:
                std::vector<double> f_heap, g_heap, h_heap;
                std::vector<int> index_heap;
                std::unordered_map<int, int> index_to_node;

                void swap(int node_a, int node_b)
                {
                        double t = f_heap[node_a];
                        f_heap[node_a] = f_heap[node_b];
                        f_heap[node_b] = t;
                        t = g_heap[node_a];
                        g_heap[node_a] = g_heap[node_b];
                        g_heap[node_b] = t;
                        t = h_heap[node_a];
                        h_heap[node_a] = h_heap[node_b];
                        h_heap[node_b] = t;
                        int index_a = index_heap[node_a], index_b = index_heap[node_b];
                        index_to_node[index_a] = node_b;
                        index_to_node[index_b] = node_a;
                        index_heap[node_a] = index_b;
                        index_heap[node_b] = index_a;
                }

        };

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}
			bool operator<(AStarPlannerNode other) const
		    {
		        return this->f < other.f;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
		        return this->f > other.f;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }

	};

	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/

			bool init_ad_star(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, double init_weight, int heuristic_index, My_heap &open_list, std::set<int> &closed_list, std::unordered_map<int, double> &g_val, std::unordered_map<int, int> &come_from_g, std::unordered_map<int, double> &rhs_val, std::unordered_map<int, int> &come_from_rhs, std::set<int> &incons_list, double &weight_output, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs, bool append_to_path = false);

			void get_changed_state_ad_star(std::set<int> &changed_state, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase);

			bool detect_significant_edge_cost_change(SteerLib::SpatialDataBaseInterface * _gSpatialDatabase);

			bool loop_ad_star(std::vector<Util::Point>& agent_path, Util::Point start, double weight_decay, int heuristic_index, My_heap &open_list, std::set<int> &closed_list, std::unordered_map<int, double> &g_val, std::unordered_map<int, int> &come_from_g, std::unordered_map<int, double> &rhs_val, std::unordered_map<int, int> &come_from_rhs, std::set<int> &incons_list, double &weight_output, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs, bool append_to_path = false);

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);

			// A4
			int node_expanded_ad_star, node_generated_ad_star, heuristic_ad_star;
			double w0_ad_star, w_decay_ad_star, w_ad_star;
			My_heap open_list_ad_star;
			std::set<int> closed_list_ad_star, incons_list_ad_star;
			std::unordered_map<int, double> g_val_ad_star, rhs_val_ad_star;
			std::unordered_map<int, int> come_from_g_ad_star, come_from_rhs_ad_star;
			Util::Point goal_ad_star;

		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	};


}


#endif
