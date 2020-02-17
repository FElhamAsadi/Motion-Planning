/* 
 Vehicle.h
 Here, a class has been defined to hold all information of a vehicle in the road
 This information will be used for path planning 
*/

#ifndef VEHICLE_H
#define VEHICLE_H

#include "json.hpp"
#include <vector>
#include <math.h>
#include "helpers.h"


using json = nlohmann::json;
using std::vector;

struct OtherVeh
{
	int id;       //The id is a unique identifier for that car
	double x;     //The x, y values are in global map coordinates
	double y;
	double vx;    //The vx, vy values are the velocity components in reference to the global map
	double vy;
	double s;     //The s and d are the Frenet coordinates for that car
	double d;
	double yaw;   //yaw angle in degree
	int lane;     //The current lane of the car
	double speed;
};


class Vehicle
{
public:
	
	//Constructor functions
    Vehicle();
	Vehicle(json &j);

	//The json message contains: [ id, x, y, vx, vy, s, d]
	double x;     //The x, y values are in global map coordinates
	double y; 	
	double s;     //The s and d are the Frenet coordinates for that car
	double d;
	double yaw;   //yaw angle in degree
	int lane;     //The current lane of the car
	double speed; 
	
	// Previous path data given to the Planner
	vector<double> previous_path_x;
	vector<double> previous_path_y;

	// Previous path's end s and d values 
	double end_path_s;
	double end_path_d;

	//Information about other vehicles						
	vector<OtherVeh> other_veh;  //other vehicles in that road

};


#endif // VEHICLE_H
