#ifndef MY_ROBOT_H
#define MY_ROBOT_H


// C++ libraries
#include <iostream>
#include <math.h>

// Webots libraries
#include <webots/Robot.hpp>
#include <webots/Motor.hpp> // Left Right Wheel
#include <webots/Camera.hpp> // Forward and Spherical Camera
#include <webots/DistanceSensor.hpp> // 16 Laser Sensors in the front of the robot
#include <webots/PositionSensor.hpp> // Encoders
#include <webots/Compass.hpp> // Compass
#include <webots/GPS.hpp>  // GPS

#define THRESHOLD 100
#define MAX_SPEED 6

using namespace webots;

class MyRobot : public Robot {
private:
    // Motors
    Motor *left_wheel_motor;
    Motor *right_wheel_motor;

    // Camera sensors
    Camera *forward_camera;
    Camera *spherical_camera;

    // Laser sensors
    DistanceSensor *laser_sensors[16];

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
    double get_laser_sensor_value(int index);

    // Encoders
    double get_left_wheel_encoder_value();
    double get_right_wheel_encoder_value();

    // Compass
    double get_compass_value();

    // GPS
    double get_gps_value();

};

#endif

