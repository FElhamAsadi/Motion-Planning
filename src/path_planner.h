/* PathPlanner.h */

#ifndef SRC_PATH_PLANNER_H_
#define SRC_PATH_PLANNER_H_

#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "helpers.h"

using std::vector;
using std::string;
using json = nlohmann::json;


class PathPlanner
{


public:

	int lane = 1;
	float ref_vel = 0.224;
    
	Vehicle ego;
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	//Constructor functions
	PathPlanner();
	PathPlanner(vector<double> Map_Waypoints_X,
		vector<double> Map_Waypoints_Y,
		vector<double> Map_Waypoints_S,
		vector<double> Map_Waypoints_dX,
		vector<double> Map_Waypoints_dY);

	void ReceiveData(json);
	
	//Safety check
	vector<bool> SafetyCheck();

	//Next action
	double NextAction(vector<bool> safety);

	//Path planning
	json PathPoints(double speed_change);

};


#endif /* SRC_PATH_PLANNER_H_ */