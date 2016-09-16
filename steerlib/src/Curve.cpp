//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI
	// Robustness: make sure there is at least two control point: start and end points
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve

	/* implement */
	if (checkRobust())
	{
		DrawLib::glColor(curveColor);
		Point startPoint, nextPoint;
		for (int i = 0; i < controlPoints.size() - 1; i++)
		{
			startPoint.x = (controlPoints[i]).position.x;
			startPoint.y = (controlPoints[i]).position.y;
			startPoint.z = (controlPoints[i]).position.z;
			for (int j = (controlPoints[i]).time + window; j <= (controlPoints[i + 1]).time; j += window)
			{
				calculatePoint(nextPoint, j);
				DrawLib::drawLine(startPoint, nextPoint, curveColor, curveThickness);
				startPoint.x = nextPoint.x;
				startPoint.y = nextPoint.y;
				startPoint.z = nextPoint.z;
			}
		}
	}
	
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	/* implement */

	for (int i = 0; i < controlPoints.size(); i++)
		for (int j = i + 1; j < controlPoints.size(); j++)
			if ((controlPoints[i]).time > (controlPoints[j]).time)
			{
				float temp;
				temp = ((controlPoints[i]).position).x;
				((controlPoints[i]).position).x = ((controlPoints[j]).position).x;
				((controlPoints[j]).position).x = temp;
                                temp = ((controlPoints[i]).position).y;
                                ((controlPoints[i]).position).y = ((controlPoints[j]).position).y;
                                ((controlPoints[j]).position).y = temp;
                                temp = ((controlPoints[i]).position).z;
                                ((controlPoints[i]).position).z = ((controlPoints[j]).position).z;
                                ((controlPoints[j]).position).z = temp;
                                temp = ((controlPoints[i]).tangent).x;
                                ((controlPoints[i]).tangent).x = ((controlPoints[j]).tangent).x;
                                ((controlPoints[j]).tangent).x = temp;
                                temp = ((controlPoints[i]).tangent).y;
                                ((controlPoints[i]).tangent).y = ((controlPoints[j]).tangent).y;
                                ((controlPoints[j]).tangent).y = temp;
                                temp = ((controlPoints[i]).tangent).z;
                                ((controlPoints[i]).tangent).z = ((controlPoints[j]).tangent).z;
                                ((controlPoints[j]).tangent).z = temp;
				temp = (controlPoints[i]).time;
				(controlPoints[i]).time = (controlPoints[j]).time;
				(controlPoints[j]).time = temp;
			}
	
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	/* implement */

	if (controlPoints.size() < 2)
		return false;

	return true;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	/* implement */

	for (nextPoint = 1; nextPoint < controlPoints.size(); nextPoint++)
		if ((controlPoints[nextPoint]).time > time)
			break;
	
	return true;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	// Calculate position at t = time on Hermite curve

	/* implement */
	float location_i, location_next, shape_i, shape_next;
	float coefficient_a, coefficient_b, coefficient_c, coefficient_d;
	float time_def, time_def_2, time_def_3, time_delta;

	time_def = (controlPoints[nextPoint]).time - (controlPoints[nextPoint - 1]).time;
	time_def_2 = time_def * time_def;
	time_def_3 = time_def_2 * time_def;
	time_delta = time - (controlPoints[nextPoint - 1]).time;

	/* x */

	location_i = (controlPoints[nextPoint - 1]).position.x;
	location_next = (controlPoints[nextPoint]).position.x;
	shape_i = (controlPoints[nextPoint - 1]).tangent.x;
	shape_next = (controlPoints[nextPoint]).tangent.x;

	coefficient_a = -2 * (location_next - location_i) / time_def_3 + (shape_i + shape_next) / time_def_2;
	coefficient_b = 3 * (location_next - location_i) / time_def_2 - (2 * shape_i + shape_next) / time_def;
	coefficient_c = shape_i;
	coefficient_d = location_i;

	newPosition.x = coefficient_a * time_delta * time_delta * time_delta + coefficient_b * time_delta * time_delta + coefficient_c * time_delta + coefficient_d;

	/* y */

        location_i = (controlPoints[nextPoint - 1]).position.y;
        location_next = (controlPoints[nextPoint]).position.y;
        shape_i = (controlPoints[nextPoint - 1]).tangent.y;
        shape_next = (controlPoints[nextPoint]).tangent.y;

        coefficient_a = -2 * (location_next - location_i) / time_def_3 + (shape_i + shape_next) / time_def_2;
        coefficient_b = 3 * (location_next - location_i) / time_def_2 - (2 * shape_i + shape_next) / time_def;
        coefficient_c = shape_i;
        coefficient_d = location_i;

        newPosition.y = coefficient_a * time_delta * time_delta * time_delta + coefficient_b * time_delta * time_delta + coefficient_c * time_delta + coefficient_d;

	/* z */

        location_i = (controlPoints[nextPoint - 1]).position.z;
        location_next = (controlPoints[nextPoint]).position.z;
        shape_i = (controlPoints[nextPoint - 1]).tangent.z;
        shape_next = (controlPoints[nextPoint]).tangent.z;

        coefficient_a = -2 * (location_next - location_i) / time_def_3 + (shape_i + shape_next) / time_def_2;
        coefficient_b = 3 * (location_next - location_i) / time_def_2 - (2 * shape_i + shape_next) / time_def;
        coefficient_c = shape_i;
        coefficient_d = location_i;

        newPosition.z = coefficient_a * time_delta * time_delta * time_delta + coefficient_b * time_delta * time_delta + coefficient_c * time_delta + coefficient_d;	

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	// Calculate position at t = time on Catmull-Rom curve
	

	/* implement */

	float location_i, location_next, shape_i, shape_next;
        float coefficient_a, coefficient_b, coefficient_c, coefficient_d;
        float time_def, time_def_2, time_def_3, time_delta;

        time_def = (controlPoints[nextPoint]).time - (controlPoints[nextPoint - 1]).time;
        time_def_2 = time_def * time_def;
        time_def_3 = time_def_2 * time_def;
        time_delta = time - (controlPoints[nextPoint - 1]).time;

        /* x */

        location_i = (controlPoints[nextPoint - 1]).position.x;
        location_next = (controlPoints[nextPoint]).position.x;
	if (nextPoint == 1)
	{
		float t2 = (controlPoints[2]).time, t1 = (controlPoints[1]).time, t0 = (controlPoints[0]).time;
		float p2 = (controlPoints[2]).position.x, p1 = (controlPoints[1]).position.x, p0 = (controlPoints[0]).position.x;
		shape_i = ((t2 - t0) / (t2 - t1)) * ((p1 - p0) / (t1 - t0)) - ((t1 - t0) / (t2 - t1))* ((p2 - p0)/ (t2 - t0));
	}
	else
	{
		float t_o = (controlPoints[nextPoint - 2]).time, t_i = (controlPoints[nextPoint - 1]).time, t_n = (controlPoints[nextPoint]).time;
		float p_o = (controlPoints[nextPoint - 2]).position.x, p_i = (controlPoints[nextPoint - 1]).position.x, p_n = (controlPoints[nextPoint]).position.x;
		shape_i = ((t_i - t_o) / (t_n - t_o)) * ((p_n - p_i) / (t_n - t_i)) + ((t_n - t_i) / (t_n - t_o)) * ((p_i - p_o) / (t_i - t_o));
	}
	if (nextPoint >= controlPoints.size() - 1)
	{
		int n_size = controlPoints.size();
		float t_1 = (controlPoints[n_size - 1]).time, t_2 = (controlPoints[n_size - 2]).time, t_3 = (controlPoints[n_size - 3]).time;
		float p_1 = (controlPoints[n_size - 1]).position.x, p_2 = (controlPoints[n_size - 2]).position.x, p_3 = (controlPoints[n_size - 3]).position.x;
		shape_next = ((t_1 - t_3) / (t_2 - t_3)) * ((p_1 - p_2) /(t_1 - t_2)) - ((t_1 - t_2) / (t_2 - t_3)) * ((p_1 - p_3) / (t_1 - t_3));
	}
	else
	{
		float t_o = (controlPoints[nextPoint - 1]).time, t_i = (controlPoints[nextPoint]).time, t_n = (controlPoints[nextPoint + 1]).time;
		float p_o = (controlPoints[nextPoint - 1]).position.x, p_i = (controlPoints[nextPoint]).position.x, p_n = (controlPoints[nextPoint + 1]).position.x;
		shape_next = ((t_i - t_o) / (t_n - t_o)) * ((p_n - p_i) / (t_n - t_i)) + ((t_n - t_i) / (t_n - t_o)) * ((p_i - p_o) / (t_i - t_o));
	}

        coefficient_a = -2 * (location_next - location_i) / time_def_3 + (shape_i + shape_next) / time_def_2;
        coefficient_b = 3 * (location_next - location_i) / time_def_2 - (2 * shape_i + shape_next) / time_def;
        coefficient_c = shape_i;
        coefficient_d = location_i;

        newPosition.x = coefficient_a * time_delta * time_delta * time_delta + coefficient_b * time_delta * time_delta + coefficient_c * time_delta + coefficient_d;

        /* y */

        location_i = (controlPoints[nextPoint - 1]).position.y;
        location_next = (controlPoints[nextPoint]).position.y;
        if (nextPoint == 1)
        {
                float t2 = (controlPoints[2]).time, t1 = (controlPoints[1]).time, t0 = (controlPoints[0]).time;
                float p2 = (controlPoints[2]).position.y, p1 = (controlPoints[1]).position.y, p0 = (controlPoints[0]).position.y;
                shape_i = ((t2 - t0) / (t2 - t1)) * ((p1 - p0) / (t1 - t0)) - ((t1 - t0) / (t2 - t1))* ((p2 - p0)/ (t2 - t0));
        }
        else
        {
                float t_o = (controlPoints[nextPoint - 2]).time, t_i = (controlPoints[nextPoint - 1]).time, t_n = (controlPoints[nextPoint]).time;
                float p_o = (controlPoints[nextPoint - 2]).position.y, p_i = (controlPoints[nextPoint - 1]).position.y, p_n = (controlPoints[nextPoint]).position.y;
                shape_i = ((t_i - t_o) / (t_n - t_o)) * ((p_n - p_i) / (t_n - t_i)) + ((t_n - t_i) / (t_n - t_o)) * ((p_i - p_o) / (t_i - t_o));
        }
        if (nextPoint >= controlPoints.size() - 1)
        {
                int n_size = controlPoints.size();
                float t_1 = (controlPoints[n_size - 1]).time, t_2 = (controlPoints[n_size - 2]).time, t_3 = (controlPoints[n_size - 3]).time;
                float p_1 = (controlPoints[n_size - 1]).position.y, p_2 = (controlPoints[n_size - 2]).position.y, p_3 = (controlPoints[n_size - 3]).position.y;
                shape_next = ((t_1 - t_3) / (t_2 - t_3)) * ((p_1 - p_2) /(t_1 - t_2)) - ((t_1 - t_2) / (t_2 - t_3)) * ((p_1 - p_3) / (t_1 - t_3));
        }
        else
        {
                float t_o = (controlPoints[nextPoint - 1]).time, t_i = (controlPoints[nextPoint]).time, t_n = (controlPoints[nextPoint + 1]).time;
                float p_o = (controlPoints[nextPoint - 1]).position.y, p_i = (controlPoints[nextPoint]).position.y, p_n = (controlPoints[nextPoint + 1]).position.y;
                shape_next = ((t_i - t_o) / (t_n - t_o)) * ((p_n - p_i) / (t_n - t_i)) + ((t_n - t_i) / (t_n - t_o)) * ((p_i - p_o) / (t_i - t_o));
        }

        coefficient_a = -2 * (location_next - location_i) / time_def_3 + (shape_i + shape_next) / time_def_2;
        coefficient_b = 3 * (location_next - location_i) / time_def_2 - (2 * shape_i + shape_next) / time_def;
        coefficient_c = shape_i;
        coefficient_d = location_i;

        newPosition.y = coefficient_a * time_delta * time_delta * time_delta + coefficient_b * time_delta * time_delta + coefficient_c * time_delta + coefficient_d;

        /* z */

        location_i = (controlPoints[nextPoint - 1]).position.z;
        location_next = (controlPoints[nextPoint]).position.z;
        if (nextPoint == 1)
        {
                float t2 = (controlPoints[2]).time, t1 = (controlPoints[1]).time, t0 = (controlPoints[0]).time;
                float p2 = (controlPoints[2]).position.z, p1 = (controlPoints[1]).position.z, p0 = (controlPoints[0]).position.z;
                shape_i = ((t2 - t0) / (t2 - t1)) * ((p1 - p0) / (t1 - t0)) - ((t1 - t0) / (t2 - t1))* ((p2 - p0)/ (t2 - t0));
        }
        else
        {
                float t_o = (controlPoints[nextPoint - 2]).time, t_i = (controlPoints[nextPoint - 1]).time, t_n = (controlPoints[nextPoint]).time;
                float p_o = (controlPoints[nextPoint - 2]).position.z, p_i = (controlPoints[nextPoint - 1]).position.z, p_n = (controlPoints[nextPoint]).position.z;
                shape_i = ((t_i - t_o) / (t_n - t_o)) * ((p_n - p_i) / (t_n - t_i)) + ((t_n - t_i) / (t_n - t_o)) * ((p_i - p_o) / (t_i - t_o));
        }
        if (nextPoint >= controlPoints.size() - 1)
        {
                int n_size = controlPoints.size();
                float t_1 = (controlPoints[n_size - 1]).time, t_2 = (controlPoints[n_size - 2]).time, t_3 = (controlPoints[n_size - 3]).time;
                float p_1 = (controlPoints[n_size - 1]).position.z, p_2 = (controlPoints[n_size - 2]).position.z, p_3 = (controlPoints[n_size - 3]).position.z;
                shape_next = ((t_1 - t_3) / (t_2 - t_3)) * ((p_1 - p_2) /(t_1 - t_2)) - ((t_1 - t_2) / (t_2 - t_3)) * ((p_1 - p_3) / (t_1 - t_3));
        }
        else
        {
                float t_o = (controlPoints[nextPoint - 1]).time, t_i = (controlPoints[nextPoint]).time, t_n = (controlPoints[nextPoint + 1]).time;
                float p_o = (controlPoints[nextPoint - 1]).position.z, p_i = (controlPoints[nextPoint]).position.z, p_n = (controlPoints[nextPoint + 1]).position.z;
                shape_next = ((t_i - t_o) / (t_n - t_o)) * ((p_n - p_i) / (t_n - t_i)) + ((t_n - t_i) / (t_n - t_o)) * ((p_i - p_o) / (t_i - t_o));
        }

        coefficient_a = -2 * (location_next - location_i) / time_def_3 + (shape_i + shape_next) / time_def_2;
        coefficient_b = 3 * (location_next - location_i) / time_def_2 - (2 * shape_i + shape_next) / time_def;
        coefficient_c = shape_i;
        coefficient_d = location_i;

        newPosition.z = coefficient_a * time_delta * time_delta * time_delta + coefficient_b * time_delta * time_delta + coefficient_c * time_delta + coefficient_d;

	// Return result
	return newPosition;
}
