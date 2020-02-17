#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "path_planner.h"
#include <math.h>
#include <chrono>
#include <thread>
#include "vehicle.h"

using namespace std;


// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
	uWS::Hub h;

	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> Map_Waypoints_X;
	vector<double> Map_Waypoints_Y;
	vector<double> Map_Waypoints_S;
	vector<double> Map_Waypoints_dX;
	vector<double> Map_Waypoints_dY;


	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";

	std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

	string line;
	while (getline(in_map_, line)) {
		std::istringstream iss(line);
		double x;
		double y;
		float s;
		float d_x;
		float d_y;
		iss >> x;
		iss >> y;
		iss >> s;
		iss >> d_x;
		iss >> d_y;
		Map_Waypoints_X.push_back(x);
		Map_Waypoints_Y.push_back(y);
		Map_Waypoints_S.push_back(s);
		Map_Waypoints_dX.push_back(d_x);
		Map_Waypoints_dY.push_back(d_y);
	}

	PathPlanner path_planner(Map_Waypoints_X, Map_Waypoints_Y, Map_Waypoints_S, Map_Waypoints_dX, Map_Waypoints_dY);

	h.onMessage(
		[&path_planner](uWS::WebSocket<uWS::SERVER> ws, char* data, size_t length,
			uWS::OpCode opCode) {			// "42" at the start of the message means there's a websocket message event.
			  // The 4 signifies a websocket message
			  // The 2 signifies a websocket event

				if (length && length > 2 && data[0] == '4' && data[1] == '2')
				{
					auto s = hasData(data);

					if (s != "") {
						auto j = json::parse(s);

						string event = j[0].get<string>();

						if (event == "telemetry") {
							// j[1] is the data JSON object

							path_planner.ReceiveData(j);
							vector<bool> Safe = path_planner.SafetyCheck();
							double speed_change = path_planner.NextAction(Safe);
							json msgJson = path_planner.PathPoints(speed_change);

							auto msg = "42[\"control\"," + msgJson.dump() + "]";
							ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
						}  // end "telemetry" if
					}
					else {
						// Manual driving
						std::string msg = "42[\"manual\",{}]";
						ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
					}
				}  // end websocket if
		}); // end h.onMessage

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
		});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
		char* message, size_t length) {
			ws.close();
			std::cout << "Disconnected" << std::endl;
		});

	int port = 4567;
	if (h.listen(port)) {
		std::cout << "Listening to port " << port << std::endl;
	}
	else {
		std::cerr << "Failed to listen to port" << std::endl;
		return -1;
	}

	h.run();
}