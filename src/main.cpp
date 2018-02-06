#include "Tools.h"
#include "EKFTracker.h"

#include "json.hpp"

#include <uWS/uWS.h>
#include <iostream>
#include <math.h>


using namespace std;
using json = nlohmann::json;


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string will be returned.
std::string hasData(std::string s) {
    auto found_null = s.find("null");
    auto b1 = s.find_first_of("[");
    auto b2 = s.find_first_of("]");

    if (found_null != std::string::npos) {
        return "";
    } else if (b1 != std::string::npos && b2 != std::string::npos) {
        return s.substr(b1, b2 - b1 + 1);
    }

    return "";
}


int main() {
    uWS::Hub h;

    // Create a EKFTracker instance
    EKFTracker ekfTracker;

    // Used to compute the RMSE later:
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // MESSAGE PROCESSING:

    h.onMessage([
        &ekfTracker,
        &tools,
        &estimations,
        &ground_truth
    ](
        uWS::WebSocket<uWS::SERVER> ws,
        char *data,
        size_t length,
        uWS::OpCode opCode
    ) {

        // "42" at the start of the message means there's a websocket message event:
        // - The 4 signifies a websocket message
        // - The 2 signifies a websocket event

        if (length && length > 2 && data[0] == '4' && data[1] == '2') {

            auto s = hasData(std::string(data));

            if (s != "") {
                auto j = json::parse(s);

                std::string event = j[0].get<std::string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    string sensor_measurment = j[1]["sensor_measurement"];

                    MeasurementPackage meas_package;
                    istringstream iss(sensor_measurment);

                    // Reads first element from the current line:
                    string sensor_type;
                    iss >> sensor_type;

                    // Filter out some values to run only on laser or radar:

                    /* if (sensor_type.compare("L") == 0) {
                        std::string msg = "42[\"manual\",{}]";

                        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                        return;
                    } */

                    // Process laser and radar data:

                    if (sensor_type.compare("L") == 0) {

                        // Set sensor type:
                        meas_package.sensor_type_ = MeasurementPackage::LASER;

                        // Read measurements:
                        float px;
                        float py;

                        iss >> px;
                        iss >> py;

                        // Set them in the package:
                        meas_package.raw_measurements_ = VectorXd(2);
                        meas_package.raw_measurements_ << px, py;
                    } else if (sensor_type.compare("R") == 0) {

                        // Set sensor type:
                        meas_package.sensor_type_ = MeasurementPackage::RADAR;

                        // Read measurements:
                        float ro;
                        float theta;
                        float ro_dot;

                        iss >> ro;
                        iss >> theta;
                        iss >> ro_dot;

                        // Set them in the package:
                        meas_package.raw_measurements_ = VectorXd(3);
                        meas_package.raw_measurements_ << ro, theta, ro_dot;
                    }

                    // Read timestamp and set it in the package:
                    long long timestamp;
                    iss >> timestamp;
                    meas_package.timestamp_ = timestamp;

                    // Read x, y, vx and vy ground truth values:
                    float x_gt;
                    float y_gt;
                    float vx_gt;
                    float vy_gt;

                    iss >> x_gt;
                    iss >> y_gt;
                    iss >> vx_gt;
                    iss >> vy_gt;

                    // Create a VectorXd with them and push them to the global ground_truth vector:
                    VectorXd gt_values(4);
                    gt_values(0) = x_gt;
                    gt_values(1) = y_gt;
                    gt_values(2) = vx_gt;
                    gt_values(3) = vy_gt;

                    ground_truth.push_back(gt_values);

                    // Call ProcessMeasurment(meas_package) for Kalman filter
                    ekfTracker.processMeasurement(meas_package);

                    // Push the current estimated x,y positon from the Kalman filter's state vector
                    VectorXd estimate(4);
                    estimate = ekfTracker.getCurrentState();

                    float px = estimate(0);
                    float py = estimate(1);

                    estimations.push_back(estimate);

                    // Calculate RMSE
                    VectorXd RMSE = tools.calculateRMSE(estimations, ground_truth);

                    // Send estimated position and RMSEs back to the simulator:
                    json msgJson;

                    // Position:
                    msgJson["estimate_x"] = px;
                    msgJson["estimate_y"] = py;

                    // RMSEs:
                    msgJson["rmse_x"] = RMSE(0);
                    msgJson["rmse_y"] = RMSE(1);
                    msgJson["rmse_vx"] = RMSE(2);
                    msgJson["rmse_vy"] = RMSE(3);

                    auto msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";

                    // Log it:
                    // std::cout << msg << std::endl;

                    // Send it:
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                std::string msg = "42[\"manual\",{}]";

                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }

    });

    // We don't need this since we're not using HTTP but if it's removed the program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";

        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // I guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!" << std::endl << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();

        std::cout << "Disconnected!" << std::endl << std::endl << std::endl;
    });

    const int port = 4567;

    if (h.listen(port)) {
        std::cout << std::endl << "Listening on port " << port << "..." << std::endl << std::endl;
    } else {
        std::cerr << std::endl << "Failed to listen on port" << port << "!" << std::endl << std::endl;

        return -1;
    }

    h.run();
}
