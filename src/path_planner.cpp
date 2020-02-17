/*
path_planner.cpp
*/

#include "path_planner.h"


//Constructor function
PathPlanner::PathPlanner()
{

}

PathPlanner::PathPlanner(vector<double> Map_Waypoints_X,
	vector<double> Map_Waypoints_Y,
	vector<double> Map_Waypoints_S,
	vector<double> Map_Waypoints_dX,
	vector<double> Map_Waypoints_dY)

{
	map_waypoints_x = Map_Waypoints_X;
	map_waypoints_y = Map_Waypoints_Y;
	map_waypoints_s = Map_Waypoints_S;
	map_waypoints_dx = Map_Waypoints_dX;
	map_waypoints_dy = Map_Waypoints_dY;
}


//Class Methods

//Reciving date about ego vehicle and other vehicles around
void PathPlanner::ReceiveData(json data)
{
	ego = Vehicle(data);
}


//Safety check
vector<bool> PathPlanner::SafetyCheck()
{
	vector<bool> safety;
	bool safety_same_lane = true;
	bool safety_left_lane = true;
	bool safety_right_lane = true;

	for (int i = 0; i < ego.other_veh.size(); i++)
	{
		int car_lane = ego.other_veh[i].lane;
		double car_speed = ego.other_veh[i].speed;
		double car_s = ego.other_veh[i].s ;

        //Checking the safety in the current lane
		if ((car_s > ego.s) && (car_s - ego.end_path_s < 30) && (car_lane == lane))  
		{
			safety_same_lane = false;
		}
       //Safety of the Left Lane Changing
		else if ((car_lane == lane -1) && (safety_left_lane == true))  
		{
			if ((car_s < ego.s) && (ego.s - car_s < 30)) //checking the safety regarding the car which is behind us in the left lane
			{
				safety_left_lane = false;
			}
			else if ((car_s > ego.s) && (car_s - ego.s <30)) //checking the safety regarding the car which is ahead of us in the left lane
			{
				safety_left_lane = false;
			}
		}
        //Safety of the Right Lane Changing
		else if ((car_lane == lane + 1) && (safety_right_lane == true))  
		{
			if ((car_s < ego.s) && (ego.s - car_s < 30))   //checking the safety regarding the car which is behind us in the right lane
			{
				safety_right_lane = false;
			}
			else if ((car_s > ego.s) && (car_s - ego.s < 30)) //checking the safety regarding the car which is ahead of us in the right lane
			{
				safety_right_lane = false;
			}
		}
	}
  
  	safety.push_back(safety_same_lane);
	safety.push_back(safety_left_lane);
	safety.push_back(safety_right_lane);

	return safety;
}



//Next action
double PathPlanner::NextAction(vector<bool> safety)
{
    double speed_change = 0.0;
	//Lane Keeping Action
	if (safety[0])
	{
		if (ref_vel <= MaxSpeedMPH - MaxAcc)
		{
            speed_change += MaxAcc;
		}
	}

	//Change to the Left Lane
	else if (safety[1] && lane>0)
	{
		lane--;
	}

    //Change to the right lane
	else if (safety[2] && lane<2)
	{
		lane++;
	}
  
   //Decrease your speed if there is no other option
	else if (speed_change <MaxAcc)
	{
	    speed_change -= MaxAcc;
	}
	return speed_change;
}



//Path planning
json PathPlanner::PathPoints(double speed_change)
{
	//the x and y values (the actual (x,y) points) of the proposed path which will be sent in a message

	// Do I have have previous points?
	// If previous path is almost empty, use the car as starting reference
	vector<double> next_x_vals;
	vector<double> next_y_vals;

	//Receiving the car data
	int prev_size = ego.previous_path_x.size();  //the last path that car was following

	//create a listof widely spaced (x,y) waypints, evenly spaced at 30m
	//later we will interpolate these pints with a spline and fill it in with more points that control speed 
	vector<double> ptsx;
	vector<double> ptsy;

	//reference x,y, yaw states
	//either we will reference the starting point as where the car is or at the previous paths end point
	double ref_x = ego.x;
	double ref_y = ego.y;
	double ref_yaw = deg2rad(ego.yaw);
	if (prev_size < 2)
	{
		// Use two points that make the path tangent to the car
		double prev_car_x = ego.x - cos(ego.yaw);
		double prev_car_y = ego.y - sin(ego.yaw);

		ptsx.push_back(prev_car_x);
		ptsx.push_back(ego.x);

		ptsy.push_back(prev_car_y);
		ptsy.push_back(ego.y);
	}
	else  //use the previous path's end point as starting reference
	{
		//Redefine reference as previous path end point 
		ref_x = ego.previous_path_x[prev_size - 1];
		ref_y = ego.previous_path_y[prev_size - 1];

		double ref_x_prev = ego.previous_path_x[prev_size - 2];
		double ref_y_prev = ego.previous_path_y[prev_size - 2];
		ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

		ptsx.push_back(ref_x_prev);
		ptsx.push_back(ref_x);

		ptsy.push_back(ref_y_prev);
		ptsy.push_back(ref_y);
	}

	// Setting up target points in the future
	//In Frenet add evenly 30m spaced points ahead of the starting refernce
	vector<double> next_wp0 = getXY(ego.s + 45, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp1 = getXY(ego.s + 75, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_wp2 = getXY(ego.s + 105, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
  
	ptsx.push_back(next_wp0[0]);
	ptsx.push_back(next_wp1[0]);
	ptsx.push_back(next_wp2[0]);

	ptsy.push_back(next_wp0[1]);
	ptsy.push_back(next_wp1[1]);
	ptsy.push_back(next_wp2[1]);

	// Making coordinates to local car coordinates.
	for (int i = 0; i < ptsx.size(); i++)
	{
		double shift_x = ptsx[i] - ref_x;
		double shift_y = ptsy[i] - ref_y;

		ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
		ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
	}

	// Create the spline.
	tk::spline s;
	s.set_points(ptsx, ptsy);

	// Output path points from previous path for continuity
	for (int i = 0; i < prev_size; i++)
	{
		next_x_vals.push_back(ego.previous_path_x[i]);
		next_y_vals.push_back(ego.previous_path_y[i]);
	}

	// Find how to break up spline points so that we travel at our desired reference velocity
	// Calculate distance y position on 30m ahead
	double target_x = 30.0;
	double target_y = s(target_x);
	double target_dist = sqrt(target_x * target_x + target_y * target_y);

	double x_add_on = 0;

	for (int i = 1; i < 50 - prev_size; i++)
	{
      	ref_vel += speed_change;

		if (ref_vel > MaxSpeedMPH)
		{
			ref_vel = MaxSpeedMPH;
		}
		else if (ref_vel <= MaxAcc)
		{
			ref_vel = MaxAcc;
		} 

		double N = target_dist / (0.02 * ref_vel / 2.24);  //rel_vel: mph to kmh
		double x_point = x_add_on + target_x / N;
		double y_point = s(x_point);

		x_add_on = x_point;

		double x_ref = x_point;
		double y_ref = y_point;

		x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
		y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

		x_point += ref_x;
		y_point += ref_y;

		next_x_vals.push_back(x_point);
		next_y_vals.push_back(y_point);
	}

	json msgJson;

	msgJson["next_x"] = next_x_vals;
	msgJson["next_y"] = next_y_vals;

	return msgJson;

}

