#include "vehicle.h"


//Constructor 
Vehicle::Vehicle()
{
  
}

Vehicle::Vehicle(json &j)
{
	//Ego car's localization Data
	x = j[1]["x"];
	y = j[1]["y"];
	s = j[1]["s"];
	d = j[1]["d"];
	yaw = j[1]["yaw"];
	speed = j[1]["speed"];
    lane = (int)(d/LaneWidth);
  
	// Previous path data  
  	auto px =  j[1]["previous_path_x"];
    auto py =  j[1]["previous_path_y"];
  	for (int i=0; i<px.size(); ++i)
  	{
		previous_path_x.push_back(px[i]);
		previous_path_y.push_back(py[i]);
    }
	// Previous path's end s and d values 
	end_path_s = j[1]["end_path_s"];
	end_path_d = j[1]["end_path_d"];

    //Information about other vehicles						
	auto sensor_fusion = j[1]["sensor_fusion"];

	for (int i = 0; i < sensor_fusion.size(); ++i)
	{
		other_veh.push_back(OtherVeh());
		other_veh[i].id = sensor_fusion[i][0];
		other_veh[i].x = sensor_fusion[i][1];
		other_veh[i].y = sensor_fusion[i][2];
		other_veh[i].vx = sensor_fusion[i][3];
		other_veh[i].vy = sensor_fusion[i][4];
		other_veh[i].s = sensor_fusion[i][5];
		other_veh[i].d = sensor_fusion[i][6];
		other_veh[i].yaw = atan2(other_veh[i].y, other_veh[i].x);
		other_veh[i].lane = (int)(other_veh[i].d / LaneWidth);
		other_veh[i].speed = sqrt(other_veh[i].vx * other_veh[i].vx + other_veh[i].vy* other_veh[i].vy);
	}
	
}