#ifndef MY_ROBOT_H_
#define MY_ROBOT_H_

/**
 * @file    MyRobot.h
 * @brief   fool_controller_final file
 *
 * @author  Blake Boyer <100531828@alumos.uc3m.es>
 * @author  Giordano Arcieri <@alumnos.uc3m.es>
 * @date    2024-04
 */

#include <iostream>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Keyboard.hpp>
#include <webots/Compass.hpp>
#include <webots/DistanceSensor.hpp>
#include <math.h>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <chrono>
#include <ctime>
#include <thread>

using namespace std;
using namespace webots;

#define GREEN_THRESHOLD_UPP 245
#define GREEN_THRESHOLD_LOW 90
#define BLUE_THRESHOLD_UPP 93
#define BLUE_THRESHOLD_LOW 70
#define RED_THRESHOLD_UPP 140
#define RED_THRESHOLD_LOW 20


#define BODY_THRESHOLD 5
#define GREEN_BUFFER 1
#define FOUND_BODY_THRESHOLD 80
#define MAX_SPEED 7
#define MED_SPEED 2
#define SLOW_SPEED 1
#define VICTIM_NUMBER 2
#define BLUE_THRESHOLD 50



#define WHEELS_DISTANCE 0.3606 //[=] meters
#define WHEEL_RADIUS 0.0825  //[=] meters

#define ENCODER_TICS_PER_RADIAN 1
#define FRONT_BACK_DIFF_TRESHOLD 10
#define SIDE_DISTANCE_SENSOR_TRESHOLD 100
#define NUMBER_OF_SENSORS 16

enum Sensors
{
    FRONT_LEFT = 0,
    FRONT_RIGHT = 15,
    SIDE_FRONT_LEFT = 3,
    SIDE_FRONT_RIGHT = 12,
    SIDE_BACK_LEFT = 4,
    SIDE_BACK_RIGHT = 11,
    FRONT_MIDDLE_LEFT = 1,
    FRONT_EDGE_LEFT = 2,
    FRONT_MIDDLE_RIGHT = 14,
    FRONT_EDGE_RIGHT = 13,
};

enum Direction
{
    LEFT = 0,
    RIGHT = 1,
};

enum Navigation_Status
{
    TURNING,
    WALL_FOLLOWING,
};

class MyRobot : public Robot
{
public:
    MyRobot();

    ~MyRobot();

    void run();

private:
    int image_width_f;
    int image_height_f;

    int image_width_s;
    int image_height_s;

    int _time_step;

    // velocities
    double _left_speed, _right_speed;

    float calc_x, calc_y, calc_theta; // [=] meters
    float _sr, _sl;                   // [=] meters

    float GPS_X, GPS_Y;
    float _theta; // [=] degrees
    float radians_OR1;
    float radians_OR2;
    float _theta_OR1;
    float _theta_OR2;
    
    // target angle (initialized as 90 degrees in run function)
    float target;
    int vic_heading;
    int total_distance;
    double angle_diff_endzone;
    double _theta_endzone;

    unsigned char green_L;
    unsigned char green_R;
 
    unsigned char red_L;
    unsigned char red_R;
    
    unsigned char blue_L;
    unsigned char blue_R;

    double percentage_green_L;
    double percentage_green_R;


    // distance sensor readings
    double front_L;
    double front_R;
    double front;

    double side_F_L;
    double side_F_R;
    double side_B_L;
    double side_B_R;
    double side_L;
    double side_R;

    double front_M_L;
    double front_all;
    double front_max;
    double front_E_L;
    double edge_L;

    double front_M_R;
    double front_E_R;
    double edge_R;

    float angle_diff;

    // true-false to switch between wall following strategies
    bool turn_option;

    int delay_counter;

    // integer to count the number of victims that have been found in endzone
    int vic_count;

    // Motors
    Motor *_left_wheel_motor;
    Motor *_right_wheel_motor;

    // Motor Position Sensor
    PositionSensor *_left_wheel_sensor;
    PositionSensor *_right_wheel_sensor;
    
    // Distance sensor
    DistanceSensor *_distance_sensor[NUMBER_OF_SENSORS];

    // GPS
    GPS *_my_gps;

    // Compass sensor
    Compass *_my_compass;

    // Camera sensors
    Camera *_forward_camera;
    Camera *_spherical_camera;

    void compute_odometry();

    /**
     * @brief Computes orientation of the robot in degrees based on the information from the compass         *
     * @return orientation of the robot in degrees
     */
    /**
     * @brief Prints in the standard output the x,y,theta coordinates of the robot.
     * This method uses the encoder resolution and the wheel radius defined in the model of the robot.
     *
     * @param tics raw value read from an encoder
     * @return meters corresponding to the tics value
     */

    void print_odometry();

    void state_change();

    float encoder_tics_to_meters(float tics);

    float angle_drive(float target, float heading);

    void right_turn();

    void left_turn();

    void right_turn_slow();

    void right_turn_slow_adj();

    void left_turn_slow();

    void left_turn_slow_adj();

    void right_turn_adj();

    void left_turn_adj();

    void forward();

    void forward_slow();

    void back();

    void approach();

    void stop();

    void wall_follower();

    void maintain_distance(int side, int ideal_distance, int margin, Direction dir);

    void get_dist_val();

    void get_gps_val();
    // void green_identifier();

    void encoder_display();

    void gps_display();

    void compass_display();

    void distance_sensor_display();

    void angle_drive();

    bool obstacle_detected();
    bool avoiding_obstacle;

    Direction turn_direction();
    Direction direction;

    void blue_identifier();
    void normalize_angle(int angle);

    void turn_flush_right();

    void turn_flush_left();

    void green_identifier();

    void get_camera_size();

    void in_endzone();

    void crossed_endline();

    void search_endzone();

    void print_found();
   
    bool should_turn(Direction dir);

    // Drives the robot toward a green obstacle if unique in frame
    void green_drive();

    void display_current_time();

    void update_global_theta();

    void navigation();

    void update_robot_data();

    void display_robot_data();

    bool robot_is_in_endzone();

    bool victims_found();

    void switch_turn();
    
    double calculate_radians_OR1();
    
    double calculate_radians_OR2();
    
    double convert_bearing_to_degrees(float radians);

    Navigation_Status navigation_status;
};
#endif
