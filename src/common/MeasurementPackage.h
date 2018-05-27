#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_


#include "Eigen-3.3/Dense"


struct MeasurementPackage {

    long long timestamp_;

    enum SensorType {
        LASER,
        RADAR
    } sensor_type_;

    Eigen::VectorXd raw_measurements_;

    Eigen::VectorXd gt_;
};


#endif /* MEASUREMENT_PACKAGE_H_ */
