#ifndef MY_ROBOT_H
#define MY_ROBOT_H

// Local libraries
#include "Logger.h"

// C++ libraries
#include <iostream>
#include <math.h>
#include <array>
#include <unordered_map>

// Webots libraries
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>          // Left Right Wheel
#include <webots/Camera.hpp>         // Forward and Spherical Camera
#include <webots/DistanceSensor.hpp> // 16 Laser Sensors in the front of the robot
#include <webots/PositionSensor.hpp> // Encoders
#include <webots/Compass.hpp>        // Compass
#include <webots/GPS.hpp>            // GPS

#define THRESHOLD 100
#define MAX_SPEED 6
#define NUMBER_OF_SENSORS 16
#define DESIRED_ANGLE 90

using namespace webots;

// This is a map where the key is the name of the laser sensor and the value is the degree that the sensor is pointing torwards
const std::unordered_map<std::string, int> LaserSensor_Degree = {
    {"bs0", -10},
    {"bs1", -20},
    {"bs2", -30},
    {"bs3", -45},
    {"bs4", -45},
    {"bs5", -60},
    {"bs6", -70},
    {"bs7", -80},
    {"bs8", 80},
    {"bs9", 70},
    {"bs10", 60},
    {"bs11", 45},
    {"bs12", 45},
    {"bs13", 30},
    {"bs14", 20},
    {"bs15", 10}};

class MyRobot : public Robot
{
private:
    // Motors
    Motor *left_wheel_motor;
    Motor *right_wheel_motor;

    // Camera sensors
    Camera *forward_camera;
    Camera *spherical_camera;

    // Laser sensors
    DistanceSensor *laser_sensors[NUMBER_OF_SENSORS];

    // Encoders
    PositionSensor *left_wheel_encoder;
    PositionSensor *right_wheel_encoder;

    // Compass
    Compass *compass;

    // GPS
    GPS *gps;

    // Time step
    int time_step;

    // Velocities
    double left_speed;
    double right_speed;

public:
    MyRobot();
    ~MyRobot();

    // Driver
    void run();

    // Motors
    void set_left_speed(double speed);
    void set_right_speed(double speed);

    // Camera sensors

    // Laser sensors
    double get_laser_index_value(int index) const;
    std::array<double, NUMBER_OF_SENSORS> get_laser_sensors_values() const;

    // Encoders
    double get_left_wheel_encoder_value() const;
    double get_right_wheel_encoder_value() const;

    // Compass
    std::array<double, 3> get_compass_value() const;

    // GPS
    std::array<double, 3> get_gps_value() const;
};

#endif
