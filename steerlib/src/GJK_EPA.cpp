#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

double dot_product(const Util::Vector &vec1, const Util::Vector &vec2)
{
	double res = 0;
	
	res = vec1.x * vec2.x + vec1.y * vec2.y + vec1.z * vec2.z;

	return res;
}

double cross_product_xz(const Util::Vector &vec1, const Util::Vector &vec2)
{
	double res = 0;
	
	res = vec1.x * vec2.z - vec1.z * vec2.x;

	return res;
}

double vector_norm(const Util::Vector &vec)
{
	return sqrt(dot_product(vec, vec));
}

bool convex_contain(const std::vector<Util::Vector> &vertices, const Util::Vector &checked_point)
{
	if (vertices.size() < 2)
		return false;
	if (vertices.size() == 2)
	{
		double d_p = dot_product(vertices[0] - checked_point, vertices[1] - checked_point);
		double c_p = cross_product_xz(vertices[0] - checked_point, vertices[1] - checked_point);
		
		return (c_p == 0 && d_p <= 0);
	}

	int i;

	for (i = 0; i < vertices.size(); i++)
	{
		int i_1, i_2;
		if (i < vertices.size() - 2)
			i_1 = i + 1, i_2 = i + 2;
		else if (i == vertices.size() - 2)
			i_1 = i + 1, i_2 = 0;
		else
			i_1 = 0, i_2 = 1;
		double c1 = cross_product_xz(checked_point - vertices[i_1], vertices[i] - vertices[i_1]);
		double c2 = cross_product_xz(checked_point - vertices[i_1], vertices[i_2] - vertices[i_1]);
		if (c1 * c2 > 0)
			return false;
	}
	return true;
}

void get_support(const std::vector<Util::Vector> &vertices,const Util::Vector &d, Util::Vector &support)
{
	int i;
	double max_dot;

	support.x = (vertices[0]).x;
	support.y = (vertices[0]).y;
	support.z = (vertices[0]).z;

	max_dot = dot_product(vertices[0], d);

	for (i = 1; i < vertices.size(); i++)
		if (dot_product(vertices[i], d) > max_dot)
		{
			max_dot = dot_product(vertices[i], d);
			support.x = (vertices[i]).x;
			support.y = (vertices[i]).y;
			support.z = (vertices[i]).z;
		}
	
}

double distance_from_point_to_line(const Util::Vector &checked_point, const Util::Vector &line_point1, const Util::Vector &line_point2)
{
	double c_p = cross_product_xz(checked_point - line_point2, line_point1 - line_point2);
	double norm_segment = vector_norm(line_point1 - line_point2);
	double res = c_p / norm_segment;

	return fabs(res);
}

double distance_from_point_to_segment(const Util::Vector &checked_point, const Util::Vector &line_point1, const Util::Vector &line_point2)
{
	double d_p1 = dot_product(checked_point - line_point1, line_point2 - line_point1);
	double d_p2 = dot_product(checked_point - line_point2, line_point1 - line_point2);

	if (d_p1 > 0 && d_p2 > 0)
		return distance_from_point_to_line(checked_point, line_point1, line_point2);
	else
	{
		double d1 = vector_norm(checked_point - line_point1);
		double d2 = vector_norm(checked_point - line_point2);
		return (d1 < d2 ? d1 : d2);
	}
}

void normal_vector_xz(const Util::Vector &checked_point, const Util::Vector &line_point1, const Util::Vector &line_point2, Util::Vector &output)
{
	double vec_x = line_point2.x - line_point1.x, vec_z = line_point2.z - line_point1.z;

	output.y = 0;
	if (vec_z == 0)
	{
		output.x = 0;
		output.z = 1;
	}
	else
	{
		output.x = 1;
		output.z = -vec_x / vec_z;
	}

	if (dot_product(line_point1 - checked_point, output) < 0)
	{
		output.x = -output.x;
		output.z = -output.z;
	}
}

void remove_points_GJK(std::vector<Util::Vector> &points_set)
{
	if (points_set.size() > 2)
	{
		int save_p1 = 0, save_p2 = points_set.size() - 1;
		int i;
		Util::Vector origin(0, 0, 0);

		for (i = 0; i < points_set.size() - 1; i++)
			if (distance_from_point_to_segment(origin, points_set[i], points_set[i + 1]) < distance_from_point_to_segment(origin, points_set[save_p1], points_set[save_p2]))
			{
				save_p1 = i;
				save_p2 = i + 1;
			}
		for (i = 0; i < points_set.size(); i++)
			if (i != save_p1 && i != save_p2)
			{
				points_set.erase(points_set.begin() + i);
				break;
			}
	}
}

void normalize_vec(Util::Vector &vec_input)
{
	double s = vector_norm(vec_input);
	vec_input.x /= s;
	vec_input.y /= s;
	vec_input.z /= s;
}

bool GJK_EPA_convex(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	double eps_checked = 0.01;
	Util::Vector v, origin(0, 0, 0);
	std::vector<Util::Vector> W_set;

	// GJK
	W_set.clear();

	v.x = (_shapeA[0]).x - (_shapeB[0]).x;
	v.y = 0;
	v.z = (_shapeA[0]).z - (_shapeB[0]).z;

	while (1)
	{
		Util::Vector a_support, b_support, w_support(0, 0, 0);
		get_support(_shapeA, -v, a_support);
		get_support(_shapeB, v, b_support);
		w_support.x = a_support.x - b_support.x;
		w_support.z = a_support.z - b_support.z;
		
		if (vector_norm(w_support - v) <= eps_checked)
		{
			return false;
		}
		if (W_set.size() == 0)
		{
			W_set.push_back(w_support);
			v.x = w_support.x;
			v.z = w_support.z;
		}
		else if (W_set.size() == 1)
		{
			W_set.push_back(w_support);
			normal_vector_xz(origin, W_set[0], W_set[1], v);
		}
		else if (W_set.size() == 2)
		{
			W_set.push_back(w_support);
			if (convex_contain(W_set, origin))
				break;
			remove_points_GJK(W_set);
			normal_vector_xz(origin, W_set[0], W_set[1], v);
		}
	}

	// EPA
	double eps_check_epa = 0.00001;
	int i;
	bool first_iteration = true;
	Util::Vector res(0, 0, 0);

	while (1)
	{
		int closest_index = W_set.size() - 1, closest_next = 0;
		double closest_dis = distance_from_point_to_segment(origin, W_set[0], W_set[W_set.size() - 1]);
		Util::Vector support_new(0, 0, 0), vec_to_closest(0, 0, 0), a_support, b_support;

		for (i = 0; i < W_set.size() - 1; i++)
		{
			if (distance_from_point_to_segment(origin, W_set[i], W_set[i + 1]) < closest_dis)
			{
				closest_index = i, closest_next = i + 1;
				closest_dis = distance_from_point_to_segment(origin, W_set[i], W_set[i + 1]);
			}
		}

		normal_vector_xz(origin, W_set[closest_index], W_set[closest_next], vec_to_closest);

		normalize_vec(vec_to_closest);
		vec_to_closest.x *= closest_dis, vec_to_closest.z *= closest_dis;
		get_support(_shapeA, vec_to_closest, a_support);
		get_support(_shapeB, -vec_to_closest, b_support);
		support_new.x = a_support.x - b_support.x;
		support_new.z = a_support.z - b_support.z;

		if (vector_norm(support_new - W_set[closest_index]) <= eps_check_epa || vector_norm(support_new - W_set[closest_next]) <= eps_check_epa)
		{
			res.x = vec_to_closest.x;
			res.z = vec_to_closest.z;
			break;
		}

		if (closest_index == W_set.size() - 1)
			W_set.push_back(support_new);
		else
			W_set.insert(W_set.begin() + closest_index + 1, support_new);
	}
	return_penetration_depth = vector_norm(res);
	return_penetration_vector.x = res.x;
	return_penetration_vector.z = res.z;
	return_penetration_vector.y = 0;
	normalize_vec(return_penetration_vector);
	
	return true;
}


//Look at the GJK_EPA.h header file for documentation and instructions
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	return GJK_EPA_convex(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB);
}



