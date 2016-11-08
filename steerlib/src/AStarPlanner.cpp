//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"

#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());

		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				if (i < 0 || i >= gSpatialDatabase->getNumCellsX() || j < 0 || j >= gSpatialDatabase->getNumCellsZ())
					continue;
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	// get the neighbor list for search
	void get_neighbor_index_list(int current_cell, std::vector<int> &neighbor_list, SteerLib::SpatialDataBaseInterface *_gSpatialDatabase)
	{
		int dx[] = {-1, 0, 1, -1, 1, -1, 0, 1}, dz[] = {-1, -1, -1, 0, 0, 1, 1, 1}, i;
		unsigned int x_cur, z_cur;
		neighbor_list.clear();
		_gSpatialDatabase->getGridCoordinatesFromIndex(current_cell, x_cur, z_cur);
		for (i = 0; i < 8; i++)
		{
			int x_next = x_cur + dx[i];
			int z_next = z_cur + dz[i];
			if (x_next >= 0 && x_next < _gSpatialDatabase->getNumCellsX() && z_next >= 0 && z_next < _gSpatialDatabase->getNumCellsZ())
			{
				neighbor_list.push_back(_gSpatialDatabase->getCellIndexFromGridCoords(x_next, z_next));
			}
		}
	}

	// get the cost from current_cell to next_cell
	double cost_between(int current_cell, int next_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		unsigned int x_cur, z_cur, x_next, z_next;
		_gSpatialDatabase->getGridCoordinatesFromIndex(current_cell, x_cur, z_cur);
		_gSpatialDatabase->getGridCoordinatesFromIndex(next_cell, x_next, z_next);
		if (abs(x_cur - x_next) + abs(z_cur - z_next) == 1)
			return 1;
		else
			return sqrt(2);
	}

	// check if an agent can go to a cell
	bool can_go_to(int current_cell, int next_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner)
	{
		return (!_gSpatialDatabase->hasAnyItems(next_cell) && (planner->canBeTraversed(next_cell)));
	}

	double h0_heuristic(Util::Point &current, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		return 0;
	}

	double h1_euclidean(Util::Point &current, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		double dx = fabs(current.x - goal.x), dz = fabs(current.z - goal.z);
		return sqrt(dx * dx + dz * dz);
	}

	double h2_manhattan(Util::Point &current, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		double dx = fabs(current.x - goal.x), dz = fabs(current.z - goal.z);
		return dx + dz;
	}

	double cal_h_by_cell(int heuristic_index, int current_cell, int goal_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		Util::Point current_point, goal_point;
		_gSpatialDatabase->getLocationFromIndex(current_cell, current_point);
		_gSpatialDatabase->getLocationFromIndex(goal_cell, goal_point);
		if (heuristic_index == 0)
			return h0_heuristic(current_point, goal_point, _gSpatialDatabase);
		else if (heuristic_index == 1)
			return h1_euclidean(current_point, goal_point, _gSpatialDatabase);
		else if (heuristic_index == 2)
			return h2_manhattan(current_point, goal_point, _gSpatialDatabase);
	}

	double cal_f_by_cell(double g_val, double w, int heuristic_index, int current_cell, int goal_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		double res = g_val + w * cal_h_by_cell(heuristic_index, current_cell, goal_cell, _gSpatialDatabase);
		return res;
	}

	void build_result_path(std::vector<Util::Point> &agent_path, std::unordered_map<int, int> come_from_list, int start_cell, int goal_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		int current_cell = goal_cell, i;
		std::vector<Util::Point> reversed_list;
		reversed_list.clear();
		while (1)
		{
			Util::Point current_location;
			_gSpatialDatabase->getLocationFromIndex(current_cell, current_location);
			reversed_list.push_back(current_location);
			if (current_cell == start_cell)
				break;
			current_cell = come_from_list[current_cell];
		}
		agent_path.clear();
		for (i = reversed_list.size() - 1; i >= 0; i--)
			agent_path.push_back(reversed_list[i]);
	}

	bool weighted_a_star(std::vector<Util::Point>& agent_path, Util::Point &start, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, double weight, int heuristic_index, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs)
	{
		// init
		auto start_time = std::chrono::steady_clock::now();
		std::unordered_map<int, int> come_from;
		My_heap open_list;
		std::unordered_map<int, double> f_val, g_val, h_val;
		std::set<int> closed_list;
		int start_cell, goal_cell;
		bool path_found = false;
		
		start_cell = _gSpatialDatabase->getCellIndexFromLocation(start);
		goal_cell = _gSpatialDatabase->getCellIndexFromLocation(goal);

		come_from.clear();
		open_list.init();
		f_val.clear();
		g_val.clear();
		h_val.clear();
		closed_list.clear();
		node_expanded = 0;
		node_generated = 1;

		g_val[start_cell] = 0;
		h_val[start_cell] = cal_h_by_cell(heuristic_index, start_cell, goal_cell, _gSpatialDatabase);
		f_val[start_cell] = cal_f_by_cell(g_val[start_cell], weight, heuristic_index, start_cell, goal_cell, _gSpatialDatabase);

		open_list.insert(start_cell, f_val[start_cell], g_val[start_cell], h_val[start_cell]);

		while (!open_list.empty())
		{
			std::vector<int> neighbor_list;
			int current_cell, i;
			double f_cur, g_cur, h_cur;
			open_list.get_root(current_cell, f_cur, g_cur, h_cur);
			open_list.pop();
			closed_list.insert(current_cell);
			node_expanded++;
			if (current_cell == goal_cell)
			{
				build_result_path(agent_path, come_from, start_cell, goal_cell, _gSpatialDatabase);
				path_found = true;
				path_cost = g_cur;
				path_length = agent_path.size();
				break;
			}
			get_neighbor_index_list(current_cell, neighbor_list, _gSpatialDatabase);
			for (i = 0; i < neighbor_list.size(); i++)
				if (can_go_to(current_cell, neighbor_list[i], _gSpatialDatabase, planner) && closed_list.find(neighbor_list[i]) == closed_list.end())
				{
					int next_cell = neighbor_list[i];
					double g_new = g_cur + cost_between(current_cell, next_cell, _gSpatialDatabase);
					if (g_val.find(next_cell) == g_val.end() || g_new < g_val[next_cell])
					{
						double f_next, g_next, h_next;
						g_next = g_new;
						h_next = cal_h_by_cell(heuristic_index, next_cell, goal_cell, _gSpatialDatabase);
						f_next = cal_f_by_cell(g_next, weight, heuristic_index, next_cell, goal_cell, _gSpatialDatabase);
						if (g_val.find(next_cell) == g_val.end())
							node_generated++;
						come_from[next_cell] = current_cell;
						g_val[next_cell] = g_next;
						f_val[next_cell] = f_next;
						h_val[next_cell] = h_next;
						if (!open_list.has_index(next_cell))
							open_list.insert(next_cell, f_next, g_next, h_next);
						else
							open_list.update(next_cell, f_next, g_next, h_next);
					}
				}
		}
		
		auto end_time = std::chrono::steady_clock::now();
		auto diff_time = end_time - start_time;
		time_secs = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count());
		time_secs /= 1000000000;

		return path_found;
	}

	bool sequential_a_star(std::vector<Util::Point>& agent_path, Util::Point &start, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, std::vector<double> &weight, std::vector<int> &heuristic_index, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs)
	{
                // init
		int heuristic_n = heuristic_index.size(), i, j;

                std::unordered_map<int, int> *come_from = new std::unordered_map<int, int>[heuristic_n];
                My_heap *open_list = new My_heap[heuristic_n];
                std::unordered_map<int, double> *f_val = new std::unordered_map<int, double>[heuristic_n], *g_val = new std::unordered_map<int, double>[heuristic_n], *h_val = new std::unordered_map<int, double>[heuristic_n];
                std::set<int> *closed_list = new std::set<int>[heuristic_n];
                int start_cell, goal_cell;
                bool path_found = false;

                auto start_time = std::chrono::steady_clock::now();
                start_cell = _gSpatialDatabase->getCellIndexFromLocation(start);
                goal_cell = _gSpatialDatabase->getCellIndexFromLocation(goal);

		node_expanded = 0;
		node_generated = 0;
		for (i = 0; i < heuristic_n; i++)
		{
			double f_start, g_start, h_start;
			(come_from[i]).clear();
                	(open_list[i]).init();
			(f_val[i]).clear();
			(g_val[i]).clear();
			(h_val[i]).clear();
			(closed_list[i]).clear();
			node_generated++;
			g_start = 0;
			h_start = cal_h_by_cell(heuristic_index[i], start_cell, goal_cell, _gSpatialDatabase);
			f_start = cal_f_by_cell(g_start, weight[i], heuristic_index[i], start_cell, goal_cell, _gSpatialDatabase);
			g_val[i][start_cell] = g_start, h_val[i][start_cell] = h_start, f_val[i][start_cell] = f_start;
			(open_list[i]).insert(start_cell, f_start, g_start, h_start);
		}

		while (!((open_list[0]).empty()))
		{
			for (i = 0; i < heuristic_n; i++)
			{
				std::vector<int> neighbor_list;
				double f0_cur, g0_cur, h0_cur;
				double fi_cur, gi_cur, hi_cur;
				double f_chosen, g_chosen, h_chosen;
				int current_cell_0, current_cell_i, current_chosen, chosen_heurisic;
				(open_list[0]).get_root(current_cell_0, f0_cur, g0_cur, h0_cur);
				(open_list[i]).get_root(current_cell_i, fi_cur, gi_cur, hi_cur);
				if (heuristic_n == 1 || fi_cur <= f0_cur)
				{
					chosen_heurisic = i;
					f_chosen = fi_cur, g_chosen = gi_cur, h_chosen = hi_cur;
					current_chosen = current_cell_i;
				}
				else
				{
					chosen_heurisic = 0;
					f_chosen = f0_cur, g_chosen = g0_cur, h_chosen = h0_cur;
					current_chosen = current_cell_0;
				}
				(open_list[chosen_heurisic]).pop();
				(closed_list[chosen_heurisic]).insert(current_chosen);
				node_expanded++;
				if (current_chosen == goal_cell)
				{
					build_result_path(agent_path, come_from[chosen_heurisic], start_cell, goal_cell, _gSpatialDatabase);
					path_found = true;
					path_cost = g_chosen;
					path_length = agent_path.size();
					break;
				}
				get_neighbor_index_list(current_chosen, neighbor_list, _gSpatialDatabase);
				for (j = 0; j < neighbor_list.size(); j++)
					if (can_go_to(current_chosen, neighbor_list[j], _gSpatialDatabase, planner) && (closed_list[chosen_heurisic]).find(neighbor_list[j]) == (closed_list[chosen_heurisic]).end())
					{
						int next_cell = neighbor_list[j];
						double g_new = g_chosen + cost_between(current_chosen, next_cell, _gSpatialDatabase);
						if ((g_val[chosen_heurisic]).find(next_cell) == (g_val[chosen_heurisic]).end() || g_new < g_val[chosen_heurisic][next_cell])
						{
							double g_next = g_new;
							double h_next = cal_h_by_cell(heuristic_index[chosen_heurisic], next_cell, goal_cell, _gSpatialDatabase);
							double f_next = cal_f_by_cell(g_next, weight[chosen_heurisic], heuristic_index[chosen_heurisic], next_cell, goal_cell, _gSpatialDatabase);
							if ((g_val[chosen_heurisic]).find(next_cell) == (g_val[chosen_heurisic]).end())
								node_generated++;
							come_from[chosen_heurisic][next_cell] = current_chosen;
							g_val[chosen_heurisic][next_cell] = g_next;
							h_val[chosen_heurisic][next_cell] = h_next;
							f_val[chosen_heurisic][next_cell] = f_next;
							if (!((open_list[chosen_heurisic]).has_index(next_cell)))
								(open_list[chosen_heurisic]).insert(next_cell, f_next, g_next, h_next);
							else
								(open_list[chosen_heurisic]).update(next_cell, f_next, g_next, h_next);
						}
					}
			}
			if (path_found)
				break;
		}

		auto end_time = std::chrono::steady_clock::now();
                auto diff_time = end_time - start_time;
                time_secs = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count());
                time_secs /= 1000000000;

		delete[] come_from;
		come_from = NULL;
		delete[] open_list;
		open_list = NULL;
		delete[] f_val;
		f_val = NULL;
		delete[] g_val;
		g_val = NULL;
		delete[] h_val;
		h_val = NULL;
		delete[] closed_list;
		closed_list = NULL;

		return path_found;
	}

	bool improve_ara_star(My_heap &open_list, std::set<int> &incons_cell, std::unordered_map<int, double> &incons_f, std::unordered_map<int, double> &incons_g, std::unordered_map<int, double> &incons_h, std::unordered_map<int, int> &incons_come_from, std::set<int> &closed_list, std::unordered_map<int, int> come_from_list, std::unordered_map<int, double> &f_val, std::unordered_map<int, double> &g_val, std::unordered_map<int, double> &h_val, double weight, int heuristic_index, int start_cell, int goal_cell, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, std::vector<Util::Point> &path_reported, int &node_expanded, int &node_generated)
	{
		int i;
		bool path_found = false;
		while (!(open_list.empty()))
		{
			double f_cur, g_cur, h_cur;
			int current_cell;
			open_list.get_root(current_cell, f_cur, g_cur, h_cur);
			if (f_val.find(goal_cell) != f_val.end() && f_val[goal_cell] < f_cur)
				break;
			open_list.pop();
			closed_list.insert(current_cell);
			node_expanded++;
			std::vector<int> neighbor_list;
			get_neighbor_index_list(current_cell, neighbor_list, _gSpatialDatabase);
			for (i = 0; i < neighbor_list.size(); i++)
				if (can_go_to(current_cell, neighbor_list[i], _gSpatialDatabase, planner))
				{
					int next_cell = neighbor_list[i];
					double g_new = g_cur + cost_between(current_cell, next_cell, _gSpatialDatabase);
					if (g_val.find(next_cell) == g_val.end() || g_new < g_val[next_cell])
					{
						double g_next = g_new;
						double h_next = cal_h_by_cell(heuristic_index, next_cell, goal_cell, _gSpatialDatabase);
						double f_next = cal_f_by_cell(g_next, weight, heuristic_index, next_cell, goal_cell, _gSpatialDatabase);
						if (closed_list.find(next_cell) == closed_list.end())
						{
							come_from_list[next_cell] = current_cell;
							f_val[next_cell] = f_next;
							g_val[next_cell] = g_next;
							h_val[next_cell] = h_next;
							if (!open_list.has_index(next_cell))
							{
								node_generated++;
								open_list.insert(next_cell, f_next, g_next, h_next);
							}
							else
								open_list.update(next_cell, f_next, g_next, h_next);
						}
						else
						{
							if (incons_cell.find(next_cell) == incons_cell.end() || g_next < incons_g[next_cell])
							{
								incons_cell.insert(next_cell);
								incons_f[next_cell] = f_next;
								incons_g[next_cell] = g_next;
								incons_h[next_cell] = h_next;
								incons_come_from[next_cell] = current_cell;
							}
						}
					}
				}

		}
		if (come_from_list.find(goal_cell) != come_from_list.end())
		{
			path_found = true;
			build_result_path(path_reported, come_from_list, start_cell, goal_cell, _gSpatialDatabase);
		}
		return path_found;
	}

	void ara_move_incons_to_open(My_heap &open_list, std::unordered_map<int, double> &f_val, std::unordered_map<int, double> &g_val, std::unordered_map<int, double> &h_val, std::unordered_map<int, int> &come_from_list, std::set<int> &incons_cell, std::unordered_map<int, double> &incons_f, std::unordered_map<int, double> &incons_g, std::unordered_map<int, double> &incons_h, std::unordered_map<int, int> &incons_come_from, SteerLib::SpatialDataBaseInterface *_gSpatialDatabase, double weight, int heuristic_index, int goal_cell)
	{
		std::set<int> cell_in_open;
		cell_in_open.clear();

		while (!(open_list.empty()))
		{
			int cell_root;
			double f_root, g_root, h_root;
			open_list.get_root(cell_root, f_root, g_root, h_root);
			open_list.pop();
			cell_in_open.insert(cell_root);
			g_val[cell_root] = g_root;
			h_val[cell_root] = cal_h_by_cell(heuristic_index, cell_root, goal_cell, _gSpatialDatabase);
			f_val[cell_root] = cal_f_by_cell(g_root, weight, heuristic_index, cell_root, goal_cell, _gSpatialDatabase);
		}

		for (auto cell_checked : incons_cell)
		{
			come_from_list[cell_checked] = incons_come_from[cell_checked];
			g_val[cell_checked] = incons_g[cell_checked];
			h_val[cell_checked] = cal_h_by_cell(heuristic_index, cell_checked, goal_cell, _gSpatialDatabase);
			f_val[cell_checked] = cal_f_by_cell(g_val[cell_checked], weight, heuristic_index, cell_checked, goal_cell, _gSpatialDatabase);
			if (cell_in_open.find(cell_checked) == cell_in_open.end())
				cell_in_open.insert(cell_checked);
		}

		for (auto cell_checked : cell_in_open)
		{
			open_list.insert(cell_checked, f_val[cell_checked], g_val[cell_checked], h_val[cell_checked]);
		}

		incons_cell.clear();
		incons_f.clear();
		incons_g.clear();
		incons_h.clear();
		incons_come_from.clear();
	}

	bool ara_star(std::vector<Util::Point>& agent_path, Util::Point &start, Util::Point &goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, double init_weight, double weight_decay, int heuristic_index, double time_limit, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs)
	{
		// init
		auto start_time = std::chrono::steady_clock::now();
		std::unordered_map<int, int> come_from, incons_come_from;
		My_heap open_list;
		std::unordered_map<int, double> f_val, g_val, h_val;
		std::unordered_map<int, double> incons_f, incons_g, incons_h;
		std::set<int> closed_list, incons_cell;
		int start_cell, goal_cell;
		bool path_found = false;
		double weight = init_weight;
		
		start_cell = _gSpatialDatabase->getCellIndexFromLocation(start);
		goal_cell = _gSpatialDatabase->getCellIndexFromLocation(goal);

		come_from.clear();
		open_list.init();
		f_val.clear();
		g_val.clear();
		h_val.clear();
		closed_list.clear();
		incons_come_from.clear();
		incons_f.clear();
		incons_g.clear();
		incons_h.clear();
		incons_cell.clear();
		node_expanded = 0;
		node_generated = 1;

		g_val[start_cell] = 0;
		h_val[start_cell] = cal_h_by_cell(heuristic_index, start_cell, goal_cell, _gSpatialDatabase);
		f_val[start_cell] = cal_f_by_cell(g_val[start_cell], weight, heuristic_index, start_cell, goal_cell, _gSpatialDatabase);

		open_list.insert(start_cell, f_val[start_cell], g_val[start_cell], h_val[start_cell]);
		path_found = improve_ara_star(open_list, incons_cell, incons_f, incons_g, incons_h, incons_come_from, closed_list, come_from, f_val, g_val, h_val, weight, heuristic_index, start_cell, goal_cell, _gSpatialDatabase, planner, agent_path, node_expanded, node_generated);
		do
		{
			auto end_time = std::chrono::steady_clock::now();
			auto diff_time = end_time - start_time;
			time_secs = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count());
			time_secs /= 1000000000;
			if (time_secs >= time_limit || weight < 1)
				break;
			weight -= weight_decay;
			if (weight < 1)
				weight = 1;
			ara_move_incons_to_open(open_list, f_val, g_val, h_val, come_from, incons_cell, incons_f, incons_g, incons_h, incons_come_from, _gSpatialDatabase, weight, heuristic_index, goal_cell);
			closed_list.clear();
			path_found = improve_ara_star(open_list, incons_cell, incons_f, incons_g, incons_h, incons_come_from, closed_list, come_from, f_val, g_val, h_val, weight, heuristic_index, start_cell, goal_cell, _gSpatialDatabase, planner, agent_path, node_expanded, node_generated);
		} while (weight > 1);

		if (path_found)
		{
			path_length = agent_path.size();
			path_cost = g_val[goal_cell];
		}

		return path_found;
	}

	void key_ad_star(int s_cell, double weight, int heuristic_index, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, int start_cell, double g_input, double rhs_input, bool g_inf, bool rhs_inf, double &key1, double &key2)
	{
		if ((g_inf && !rhs_inf) || (!g_inf && !rhs_inf && g_input > rhs_input))
		{
			key1 = cal_f_by_cell(rhs_input, weight, heuristic_index, s_cell, start_cell, _gSpatialDatabase);
			key2 = rhs_input;
		}
		else if (!g_inf)
		{
			key1 = cal_f_by_cell(g_input, 1, heuristic_index, s_cell, start_cell, _gSpatialDatabase);
			key2 = g_input;
		}
	}

	void update_state_ad_star(int s_cell, double weight, int heuristic_index, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, std::set<int> &closed_list, int start_cell, int goal_cell, std::unordered_map<int, double> &rhs_val, std::unordered_map<int, double> &g_val, My_heap &open_list, std::unordered_map<int, int> &come_from_rhs, std::unordered_map<int, int> &come_from_g, std::set<int> &incons_list, int &node_expanded, int &node_generated)
	{
		int i;
		if (s_cell != goal_cell)
		{
			std::vector<int> neighbor_list;
                        get_neighbor_index_list(s_cell, neighbor_list, _gSpatialDatabase);
			double rhs_min;
			int rhs_min_come_from;
			bool rhs_found = false;
			for (i = 0; i < neighbor_list.size(); i++)
			{
				if (can_go_to(s_cell, neighbor_list[i], _gSpatialDatabase, planner))
				{
					int next_cell = neighbor_list[i];
					if (g_val.find(next_cell) == g_val.end())
						continue;
					double rhs_new = g_val[next_cell] + cost_between(s_cell, next_cell, _gSpatialDatabase);
					if (!rhs_found || (rhs_new < rhs_min))
					{
						rhs_found = true;
						rhs_min = rhs_new;
						rhs_min_come_from = next_cell;
					}
				}
			}
			if (!rhs_found)
			{
				if (come_from_rhs.find(s_cell) != come_from_rhs.end())
				{
					come_from_rhs.erase(s_cell);
					rhs_val.erase(s_cell);
				}
			}
			else
			{
				rhs_val[s_cell] = rhs_min;
				come_from_rhs[s_cell] = rhs_min_come_from;
			}
		}
		if (open_list.has_index(s_cell))
			open_list.remove_cell_by_index(s_cell);
		bool g_inf = (g_val.find(s_cell) == g_val.end()), rhs_inf = (rhs_val.find(s_cell) == rhs_val.end());
		double g_s, rhs_s;
		if (!g_inf)
			g_s = g_val[s_cell];
		if (!rhs_inf)
			rhs_s = rhs_val[s_cell];
		if ((g_inf && !rhs_inf) || (!g_inf & !rhs_inf && rhs_s != g_s))
		{
			if (closed_list.find(s_cell) == closed_list.end())
			{
				double key1, key2;
				key_ad_star(s_cell, weight, heuristic_index, _gSpatialDatabase, planner, start_cell, g_s, rhs_s, g_inf, rhs_inf, key1, key2);
				open_list.insert(s_cell, key1, key2, 0);
				node_generated++;
			}
			else
			{
				incons_list.insert(s_cell);
			}
		}
	}

	bool compute_improve_path_ad_star(SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, int start_cell, int goal_cell, double weight, int heuristic_index, My_heap &open_list, std::set<int> &closed_list, std::unordered_map<int, double> &g_val, std::unordered_map<int, double> &rhs_val, std::unordered_map<int, int> &come_from_g, std::unordered_map<int, int> &come_from_rhs, std::set<int> &incons_list, std::vector<Util::Point>& agent_path, double &path_cost, int &path_length, int &node_expanded, int &node_generated)
	{
		bool path_found = false;
		while (!(open_list.empty()))
		{
			bool g_inf_start = (g_val.find(start_cell) == g_val.end()), rhs_inf_start = (rhs_val.find(start_cell) == rhs_val.end());
			double g_start, rhs_start;
			if (!g_inf_start)
				g_start = g_val[start_cell];
			if (!rhs_inf_start)
				rhs_start = rhs_val[start_cell];
			bool rhs_start_eq_g_start = (!g_inf_start && !rhs_inf_start && g_start == rhs_start);
			double start_key1, start_key2;
			key_ad_star(start_cell, weight, heuristic_index, _gSpatialDatabase, planner, start_cell, g_start, rhs_start, g_inf_start, rhs_inf_start, start_key1, start_key2);
			int s_cell, i;
			double s_key1, s_key2, t;
			open_list.get_root(s_cell, s_key1, s_key2, t);
			bool s_better_than_start = ((g_inf_start && rhs_inf_start) || (My_heap::better(s_key1, s_key2, start_key1, start_key2)));
			if (rhs_start_eq_g_start && !s_better_than_start)
				break;
			open_list.pop();
			node_expanded++;
			bool s_inf_g = (g_val.find(s_cell) == g_val.end()), s_inf_rhs(rhs_val.find(s_cell) == rhs_val.end());
			double s_g, s_rhs;
			if (!s_inf_g)
				s_g = g_val[s_cell];
			if (!s_inf_rhs)
				s_rhs = rhs_val[s_cell];
			std::vector<int> should_update;
			get_neighbor_index_list(s_cell, should_update, _gSpatialDatabase);
			if ((s_inf_g && !s_inf_rhs) || (!s_inf_g && !s_inf_rhs && s_rhs < s_g))
			{
				g_val[s_cell] = rhs_val[s_cell];
				come_from_g[s_cell] = come_from_rhs[s_cell];
				closed_list.insert(s_cell);
			}
			else
			{
				g_val.erase(s_cell);
				come_from_g.erase(s_cell);
				should_update.push_back(s_cell);
			}
			for (i = 0; i < should_update.size(); i++)
			{
				s_cell = should_update[i];
				update_state_ad_star(s_cell, weight, heuristic_index, _gSpatialDatabase, planner, closed_list, start_cell, goal_cell, rhs_val, g_val, open_list, come_from_rhs, come_from_g, incons_list, node_expanded, node_generated);
			}
		}

		if (come_from_g.find(start_cell) != come_from_g.end())
		{
			path_found = true;
			agent_path.clear();
			int current_cell = start_cell;
			while (1)
			{
				Util::Point current_location;
				_gSpatialDatabase->getLocationFromIndex(current_cell, current_location);
				agent_path.push_back(current_location);
				if (current_cell == goal_cell)
					break;
				if (come_from_g.find(current_cell) == come_from_g.end())
				{
					path_found = false;
					break;
				}
				current_cell = come_from_g[current_cell];
			}
			if (path_found)
			{
				path_cost = g_val[start_cell];
				path_length = agent_path.size();
			}
		}

		return path_found;
	}

	void move_incons_to_open_ad_star(My_heap &open_list, std::set<int> &incons_list, double weight, int heuristic_index, std::unordered_map<int, double> &g_val, std::unordered_map<int, double> &rhs_val, std::unordered_map<int, int> &come_from_g, std::unordered_map<int, int> &come_from_rhs, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, AStarPlanner *planner, int start_cell, int goal_cell)
	{
		std::set<int> cell_insert;
		cell_insert.clear();
		while (!(open_list.empty()))
		{
			int cell_root;
			double root_key1, root_key2, root_key3;
			open_list.get_root(cell_root, root_key1, root_key2, root_key3);
			cell_insert.insert(cell_root);
			open_list.pop();
		}
		for (auto cell_incons : incons_list)
		{
			cell_insert.insert(cell_incons);
		}
		incons_list.clear();
		for (auto cell_checked : cell_insert)
		{
			bool g_inf = (g_val.find(cell_checked) == g_val.end()), rhs_inf = (rhs_val.find(cell_checked) == rhs_val.end());
			double g_check, rhs_check;
			if (!g_inf)
				g_check = g_val[cell_checked];
			if (!rhs_inf)
				rhs_check = rhs_val[cell_checked];
			if (!g_inf || !rhs_inf)
			{
				double key1_check, key2_check;
				key_ad_star(cell_checked, weight, heuristic_index, _gSpatialDatabase, planner, start_cell, g_check, rhs_check, g_inf, rhs_inf, key1_check, key2_check);
				open_list.insert(cell_checked, key1_check, key2_check, 0);
			}
		}
	}

	bool AStarPlanner::init_ad_star(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, double init_weight, int heuristic_index, My_heap &open_list, std::set<int> &closed_list, std::unordered_map<int, double> &g_val, std::unordered_map<int, int> &come_from_g, std::unordered_map<int, double> &rhs_val, std::unordered_map<int, int> &come_from_rhs, std::set<int> &incons_list, double &weight_output, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs, bool append_to_path)
	{
		goal_ad_star = goal;
		gSpatialDatabase = _gSpatialDatabase;
		weight_output = init_weight;
		g_val.clear();
		come_from_g.clear();
		rhs_val.clear();
		come_from_rhs.clear();
		closed_list.clear();
		incons_list.clear();
		open_list.clear();
		open_list.init();
		node_expanded = 0;
		node_generated = 0;
		int start_cell = _gSpatialDatabase->getCellIndexFromLocation(start);
		int goal_cell = _gSpatialDatabase->getCellIndexFromLocation(goal);
		rhs_val[goal_cell] = 0;
		double goal_key1, goal_key2;
		std::vector<Util::Point> reported_path;
		auto start_time = std::chrono::steady_clock::now();

		key_ad_star(goal_cell, init_weight, heuristic_index, _gSpatialDatabase, this, start_cell, 0, 0, true, false, goal_key1, goal_key2);
		open_list.insert(goal_cell, goal_key1, goal_key2, 0);
		node_generated++;
		bool path_found = compute_improve_path_ad_star(_gSpatialDatabase, this, start_cell, goal_cell, init_weight, heuristic_index, open_list, closed_list, g_val, rhs_val, come_from_g, come_from_rhs, incons_list, reported_path, path_cost, path_length, node_expanded, node_generated);
		auto end_time = std::chrono::steady_clock::now();
                auto diff_time = end_time - start_time;
                time_secs = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count());
                time_secs /= 1000000000;

		if (path_found)
		{
			if (!append_to_path)
				agent_path.clear();
			int i;
			for (i = 0; i < reported_path.size(); i++)
				agent_path.push_back(reported_path[i]);
		}
		return path_found;
	}

	void AStarPlanner::get_changed_state_ad_star(std::set<int> &changed_state, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		int x, z;
		changed_state.clear();
		/*
		for (x = 0; x < _gSpatialDatabase->getNumCellsX(); x++)
			for (z = 0; z < _gSpatialDatabase->getNumCellsZ(); z++)
			{
				int cell_index = _gSpatialDatabase->getCellIndexFromGridCoords(x, z);
				if (can_go_to(0, cell_index, _gSpatialDatabase, this))
					changed_state.insert(cell_index);
			}
		*/
	}

	bool AStarPlanner::detect_significant_edge_cost_change(SteerLib::SpatialDataBaseInterface * _gSpatialDatabase)
	{
		return true;
	}

	bool AStarPlanner::loop_ad_star(std::vector<Util::Point>& agent_path, Util::Point start, double weight_decay, int heuristic_index, My_heap &open_list, std::set<int> &closed_list, std::unordered_map<int, double> &g_val, std::unordered_map<int, int> &come_from_g, std::unordered_map<int, double> &rhs_val, std::unordered_map<int, int> &come_from_rhs, std::set<int> &incons_list, double &weight_output, double &path_cost, int &path_length, int &node_expanded, int &node_generated, double &time_secs, bool append_to_path)
	{
		Util::Point goal = goal_ad_star;
		SteerLib::SpatialDataBaseInterface *_gSpatialDatabase = gSpatialDatabase;
		gSpatialDatabase = _gSpatialDatabase;
		auto start_time = std::chrono::steady_clock::now();
		int start_cell = _gSpatialDatabase->getCellIndexFromLocation(start);
		int goal_cell = _gSpatialDatabase->getCellIndexFromLocation(goal);
		bool path_found = false, no_change = false;
		std::set<int> changed_state;
		std::vector<Util::Point> reported_path;
		get_changed_state_ad_star(changed_state, _gSpatialDatabase);
		for (auto state_should_update : changed_state)
		{
			update_state_ad_star(state_should_update, weight_output, heuristic_index, _gSpatialDatabase, this, closed_list, start_cell, goal_cell, rhs_val, g_val, open_list, come_from_rhs, come_from_g, incons_list, node_expanded, node_generated);
		}
		
		if (detect_significant_edge_cost_change(_gSpatialDatabase))
		{
			path_found = init_ad_star(reported_path, start, goal, _gSpatialDatabase, w0_ad_star, heuristic_index, open_list, closed_list, g_val, come_from_g, rhs_val, come_from_rhs, incons_list, weight_output, path_cost, path_length, node_expanded, node_generated, time_secs, false);
			no_change = true;
		}
		else
		{
			double weight_old = weight_output;
			weight_output -= weight_decay;
			if (weight_output < 1)
				weight_output = 1;
			no_change = (weight_old == weight_output);
		}

		if (!no_change)
		{
			move_incons_to_open_ad_star(open_list, incons_list, weight_output, heuristic_index, g_val, rhs_val, come_from_g, come_from_rhs, _gSpatialDatabase, this, start_cell, goal_cell);
			closed_list.clear();
			compute_improve_path_ad_star(_gSpatialDatabase, this, start_cell, goal_cell, weight_output, heuristic_index, open_list, closed_list, g_val, rhs_val, come_from_g, come_from_rhs, incons_list, reported_path, path_cost, path_length, node_expanded, node_generated);
		}

		if (come_from_g.find(start_cell) != come_from_g.end())
		{
			path_found = true;
			int current_cell = start_cell, i;
			reported_path.clear();
			while (1)
			{
				Util::Point current_location;
				_gSpatialDatabase->getLocationFromIndex(current_cell, current_location);
				reported_path.push_back(current_location);
				if (current_cell == goal_cell)
					break;
				if (come_from_g.find(current_cell) == come_from_g.end())
				{
					path_found = false;
					break;
				}
				current_cell = come_from_g[current_cell];
			}
			if (path_found)
			{
				path_cost = g_val[start_cell];
				if (!append_to_path)
					agent_path.clear();
				for (i = 0; i < reported_path.size(); i++)
					agent_path.push_back(reported_path[i]);
				path_length = agent_path.size();
			}
		}

		auto end_time = std::chrono::steady_clock::now();
                auto diff_time = end_time - start_time;
                time_secs = (double)(std::chrono::duration_cast<std::chrono::nanoseconds>(diff_time).count());
                time_secs /= 1000000000;

		return path_found;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		
		double path_cost, time_secs;
		int path_length, node_expanded, node_generated, i;
		bool path_found;
		std::vector<Util::Point> plan_output;
		plan_output.clear();

		// weighted A*
	
		path_found = weighted_a_star(plan_output, start, goal, _gSpatialDatabase, this, 1, 1, path_cost, path_length, node_expanded, node_generated, time_secs);
		if (!append_to_path)
		{
			agent_path.clear();
		}
		for (i = 0; i < plan_output.size(); i++)
			agent_path.push_back(plan_output[i]);


		// sequential A*
		/*
		std::vector<int> heuristic_index = {2, 1};
		std::vector<double> weight = {1, 1.1};
		path_found = sequential_a_star(plan_output, start, goal, _gSpatialDatabase, this, weight, heuristic_index, path_cost, path_length, node_expanded, node_generated, time_secs);
		if (!append_to_path)
		{
			agent_path.clear();
		}
		for (i = 0; i < plan_output.size(); i++)
			agent_path.push_back(plan_output[i]);
		*/

		// ARA*
		/*
		path_found = ara_star(plan_output, start, goal, _gSpatialDatabase, this, 3, 0.3, 1, 1, path_cost, path_length, node_expanded, node_generated, time_secs);
		printf("debug: plan done within %lf\n", time_secs);
		if (!append_to_path)
		{
			agent_path.clear();
		}
		for (i = 0; i < plan_output.size(); i++)
			agent_path.push_back(plan_output[i]);
		*/

		return path_found;
	}
}




