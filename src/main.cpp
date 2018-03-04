#include "Tools.h"
#include "EKFTracker.h"

#include "json.hpp"

#include <uWS/uWS.h>
#include <math.h>
#include <time.h>
#include <iostream>
#include <math.h>
#include <time.h>


#define SENSOR 'B'

#define RMSE_X_LIMIT 0.11
#define RMSE_Y_LIMIT 0.11
#define RMSE_VX_LIMIT 0.52
#define RMSE_VY_LIMIT 0.52

#define C_RED "\033[38;1;31m"
#define C_GREEN "\033[38;0;32m"
#define C_RST "\033[0;m"


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
    EKFTracker tracker; // TODO: This can use the abstract class type

    // Used to compute the RMSE later:
    Tools tools;
    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

    // To measure execution time:
    long double time = 0;
    int points = 0;

    // MESSAGE PROCESSING:

    h.onMessage([
        &time,
        &points,
        &tracker,
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

            const auto s = hasData(std::string(data));

            if (s != "") {
                const auto j = json::parse(s);

                std::string event = j[0].get<std::string>();

                if (event == "telemetry") {
                    // j[1] is the data JSON object

                    const string sensor_measurment = j[1]["sensor_measurement"];

                    MeasurementPackage meas_package;
                    istringstream iss(sensor_measurment);

                    // Reads first element from the current line:
                    string sensor_type;
                    iss >> sensor_type;

                    // Process laser and/or radar data:
                    // The compiler will remove the unused code blocks.

                    if (SENSOR != 'R' && sensor_type.compare("L") == 0) {

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
                    } else if (SENSOR != 'L' && sensor_type.compare("R") == 0) {

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

                    // Measure start time:
                    clock_t begin = clock();

                    // Call ProcessMeasurment(meas_package) for Kalman filter
                    vector<double> NIS = tracker.processMeasurement(meas_package);

                    // Push the current estimated x,y positon from the Kalman filter's state vector
                    VectorXd state = tracker.getCurrentState();

                    // Measure end time:
                    clock_t end = clock();

                    // Update average time:
                    time += (long double)(end - begin) / CLOCKS_PER_SEC;

                    // Get px and py and save the state:
                    const float px = state(0);
                    const float py = state(1);

                    estimations.push_back(state);

                    // Calculate RMSE
                    VectorXd RMSE = tools.calculateRMSE(estimations, ground_truth);
                    
                    const float RMSE_X = RMSE(0);
                    const float RMSE_Y = RMSE(1);
                    const float RMSE_VX = RMSE(2);
                    const float RMSE_VY = RMSE(3);

                    // Print stats:

                    if (points++ % 10 == 0) {
                        cout
                            << "                  │                                    │" << endl
                            << "     #  S    TIME │  RMSE X   RMSE Y  RMSE VX  RMSE VY │     NIS   NIS 95   NIS 90   NIS 10    NIS 5" << endl;
                    }

                    cout
                        << "  " << setfill(' ') << setw(4) << points
                        << "  " << sensor_type
                        << setprecision(0) << fixed
                        << setfill(' ') << setw(5) << 1000000 * time / points << " us"
                        << " │ "
                        << setprecision(3) << fixed
                        << "  " << (RMSE_X > RMSE_X_LIMIT ? C_RED : C_GREEN) << RMSE_X
                        << "    " << (RMSE_Y > RMSE_Y_LIMIT ? C_RED : C_GREEN) << RMSE_Y
                        << "    " << (RMSE_VX > RMSE_VX_LIMIT ? C_RED : C_GREEN) << RMSE_VX
                        << "    " << (RMSE_VY > RMSE_VY_LIMIT ? C_RED : C_GREEN) << RMSE_VY
                        << C_RST << " │ "
                        << setprecision(3) << fixed
                        << setfill(' ') << setw(7) << NIS[0] << "  "
                        << setprecision(1) << fixed
                        << setfill(' ') << setw(5) << NIS[1] << " %  "
                        << setfill(' ') << setw(5) << NIS[2] << " %  "
                        << setfill(' ') << setw(5) << NIS[3] << " %  "
                        << setfill(' ') << setw(5) << NIS[4] << " %" << endl;

                    // Send estimated position and RMSEs back to the simulator:
                    json msgJson;

                    // Position:
                    msgJson["estimate_x"] = px;
                    msgJson["estimate_y"] = py;

                    // RMSEs:
                    msgJson["rmse_x"] = RMSE_X;
                    msgJson["rmse_y"] = RMSE_Y;
                    msgJson["rmse_vx"] = RMSE_VX;
                    msgJson["rmse_vy"] = RMSE_VY;

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

        cout
            << endl
            << " Connected!" << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl
            << endl;

    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
        ws.close();

        std::cout << "Disconnected!" << std::endl << std::endl << std::endl;
    });

    const int port = 4567;

    if (h.listen(port)) {

        cout
            << endl
            << " Listening on port " << port << "..." << endl
            << endl
            << "──────────────────────────────────────────────────────" << endl;

    } else {
        std::cerr << std::endl << "Failed to listen on port" << port << "!" << std::endl << std::endl;

        return -1;
    }

    h.run();
}
