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

#define GREEN_THRESHOLD 50
#define YELLOW_THRESHOLD 110
#define BODY_THRESHOLD 5
#define GREEN_BUFFER 10
#define FOUND_BODY_THRESHOLD 80
#define MAX_SPEED 7
#define MED_SPEED 3
#define SLOW_SPEED 0.5
#define VICTIM_NUMBER 2

#define WHEELS_DISTANCE 0.32   //[=] meters
#define WHEEL_RADIUS    0.0825 //[=] meters

#define ENCODER_TICS_PER_RADIAN 1
#define NUM_DISTANCE_SENSOR 10 
#define NUMBER_OF_SENSORS 16

enum Sensors {
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

enum Direction {
    LEFT = 0,
    RIGHT = 1,
};

class MyRobot : public Robot {
public:
    /**
     * @brief Empty constructor of the class.
     */
    MyRobot();

    /**
     * @brief Destructor of the class.
     */
    ~MyRobot();

    /**
     * @brief Function with the logic of the controller.
     * @param
     * @return
     */
    void run();

    /**
     * @brief Prints in the standard output the x,y,theta coordinates of the robot.
     */
    void print_odometry();

    /**
     * @brief Checks whether the robot has reached the goal established for this controller.
     * @return true if the robot has reached the goal established for this controller; false otherwise.
     */
    bool goal_reached();
    

private:
    int image_width_f;
    int image_height_f;
    
    int image_width_s;
    int image_height_s;
    
    unsigned char green_L, red_L, blue_L;
    unsigned char green_R, red_R, blue_R;
    unsigned char green_S, red_S, blue_S;
    double percentage_green_L;
    double percentage_green_R;
    double percentage_yellow;
    
    int _time_step;

    // velocities
    double _left_speed, _right_speed;
        
    float calc_x, calc_y, calc_theta;   // [=] meters
    float _sr, _sl;  // [=] meters
    
    float GPS_X, GPS_Y;
    float _theta, _theta_goal;   // [=] rad
    
    // target angle (initialized as 90 degrees in run function)
    float target; 
    
    // variable used to control wall following strategy for head-on obstacles
    int count_turn;
    
    //distance sensor readings
    double front_L;
    double front_R;
    double front;
    
    double side_F_L;
    double side_F_R;
    double side_B_L;
    double side_B_R;
    
    double front_M_L;
    double front_E_L;
    double edge_L;
    
    double front_M_R;
    double front_E_R;
    double edge_R;

    //
    bool turn_option;

    float angle_diff;
    
    //variable to track number of times the robot has crossed
    int cross_count;
    
    //true-false to be used to alter heading direction and return to beginning
    bool found_vics;
    
    //integer to count the number of victims that have been found in endzone
    int vic_count;
    
    //variable to store heading of first victim
    int vic_heading;
    
    //true-false test variable for returning to start point after reaching endzone
    bool endzone_test;
    
    //ticker to help store goal heading for 90 degree turn
    bool tick;
    
    //ninety degree turn angle storage
    float store_ninety;
    
    double target_90;
    
    bool has_turned;
    
    //three hundred and sixty degree turn angle storage
    float store_threesixty;
    
    int delay_counter;
    
    // Motor Position Sensor
    PositionSensor* _left_wheel_sensor;
    PositionSensor* _right_wheel_sensor;

    
    // Compass sensor
    Compass * _my_compass;

    // Motors
    Motor* _left_wheel_motor;
    Motor* _right_wheel_motor;

    // Distance sensor 
    DistanceSensor * _distance_sensor[NUMBER_OF_SENSORS];
    
    //GPS 
    GPS * _my_gps;
    
    // Camera sensors
    Camera *_forward_camera;
    Camera *_spherical_camera;
    
    void compute_odometry();
        
 	/**
        * @brief Computes orientation of the robot in degrees based on the information from the compass         * 
        * @return orientation of the robot in degrees 
        */       
    double convert_bearing_to_degrees();
    //double convert_bearing_to_degrees();
    /**
         * @brief Prints in the standard output the x,y,theta coordinates of the robot. 
         * This method uses the encoder resolution and the wheel radius defined in the model of the robot.
         * 
         * @param tics raw value read from an encoder
         * @return meters corresponding to the tics value 
         */
    void state_change();
    
    float encoder_tics_to_meters(float tics);
    
    float angle_drive(float target, float heading);
    
    void right_turn();
    
    void left_turn();
    
    void right_turn_adj();
    
    void left_turn_adj();
    
    void forward();
    
    void back();
    
    void approach();
    
    void stop();
    
    void wall_follower();
    
    void get_dist_val();
    
    void get_gps_val();
    // void green_identifier();
   
    void set_velo();
    
    void encoder_display();

    void gps_display();
    
    void angle_drive();
    
    bool obstacle_detected();
    bool avoiding_obstacle;

    Direction turn_direction();
    Direction direction;

    void normalize_angle(int angle);
    
    void right_turn_ninety(double target_heading);
    
    double target_angle_90(double angle_i);
    
   // double storeval_ninetyR();
   
    void turn_flush_right();
    
    void turn_flush_left();
    
    void green_identifier();
    
    void get_camera_size();
    
    void in_endzone();
    
    void crossed_endline();
    
    void search_endzone();
    
    void print_found();
    
    //Drives the robot toward a green obstacle if unique in frame
    void green_drive();
    
    void display_current_time();
    
    void update_global_theta();
    
    void navigation();

    void update_robot_data();

    void display_robot_data();

    bool robot_is_in_endzone();

    bool victims_found();

    void switch_turn();
};
#endif

