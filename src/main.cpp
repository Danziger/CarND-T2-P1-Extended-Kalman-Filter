#include "EKFTracker.h"

#include "common/JSON-Lohmann-2.1.1/json.hpp"
#include "common/format.h"
#include "common/helpers.h"

#include <uWS/uWS.h>
#include <math.h>
#include <iostream>


#define SENSOR 'B'


// For convenience:
using json = nlohmann::json;
using namespace std;


// MAIN:

int main() {

    // Create a EKFTracker instance
    EKFTracker tracker; // TODO: This can use the abstract class type

    // MESSAGE PROCESSING:
    
    uWS::Hub h; // Initialize WebSocket.

    h.onMessage([ &tracker ](
        uWS::WebSocket<uWS::SERVER> ws,
        char *data,
        size_t length,
        uWS::OpCode opCode
    ) {

        // "42" at the start of the message means there's a websocket message event:
        // - The 4 signifies a websocket message
        // - The 2 signifies a websocket event
        const string sdata = string(data).substr(0, length);

        if (sdata.size() <= 2 || sdata[0] != '4' || sdata[1] != '2') {
            return;
        }

        const string s = helpers::hasData(sdata);

        if (s == "") {
            // Manual driving:

            std::string msg = "42[\"manual\",{}]";

            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            return;
        }

        const json j = json::parse(s);
        const string event = j[0].get<string>();

        if (event != "telemetry") {
            return;
        }

        // j[1] is the data JSON object:

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

        // Read timestamp, x, y, vx and vy ground truth values:

        long long timestamp;
        float x_gt;
        float y_gt;
        float vx_gt;
        float vy_gt;

        iss >> timestamp;
        iss >> x_gt;
        iss >> y_gt;
        iss >> vx_gt;
        iss >> vy_gt;

        // Set them in the package:

        meas_package.timestamp_ = timestamp;
        meas_package.gt_ = VectorXd(4);
        meas_package.gt_ << x_gt, y_gt, vx_gt, vy_gt;

        // Process the current measurement:
        tracker.processMeasurement(meas_package);

        // Push the current estimated x,y positon from the Kalman filter's state vector
        const VectorXd state = tracker.getCurrentState();

        // Get the current RMSE:
        VectorXd RMSE = tracker.getCurrentRMSE();

        // Get all individual components from the current state and RMSE:

        const double px = state(0);
        const double py = state(1);

        const double RMSE_X = RMSE(0);
        const double RMSE_Y = RMSE(1);
        const double RMSE_VX = RMSE(2);
        const double RMSE_VY = RMSE(3);

        // Send estimated position and RMSEs back to the simulator:

        const json msgJson = {
            { "estimate_x", px      },
            { "estimate_y", py      },
            { "rmse_x",     RMSE_X  },
            { "rmse_y",     RMSE_Y  },
            { "rmse_vx",    RMSE_VX },
            { "rmse_vy",    RMSE_VY }
        };

        const string msg = "42[\"estimate_marker\"," + msgJson.dump() + "]";

        // Send it:
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
    });

    // ON HTTP REQUEST:
    // We don't need this since we're not using HTTP but if it's removed the program doesn't compile :-(
    
    h.onHttpRequest([](
        uWS::HttpResponse *res,
        uWS::HttpRequest req,
        char *data,
        size_t,
        size_t
    ) {
        const std::string s = "<h1>Hello world!</h1>";

        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // I guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    // ON CONNECTION:

    h.onConnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        uWS::HttpRequest req
    ) {
        cout
            << endl
            << " Connected!" << endl
            << endl
            << SEPARATOR << endl
            << endl;
    });

    // ON DISCONNECTION:

    h.onDisconnection([&h](
        uWS::WebSocket<uWS::SERVER> ws,
        int code,
        char *message,
        size_t length
    ) {
        ws.close();

        cout
            << endl
            << "Disconnected!" << endl
            << endl
            << SEPARATOR << endl;
    });

    // START LISTENING:

    const int port = 4567;

    if (h.listen(port)) {

        cout
            << endl
            << " Listening on port " << port << "..." << endl
            << endl
            << SEPARATOR << endl;

    } else {

        cerr
            << endl
            << "Failed to listen on port" << port << "!"
            << endl
            << SEPARATOR << endl;

        return -1;
    }

    h.run();
}
