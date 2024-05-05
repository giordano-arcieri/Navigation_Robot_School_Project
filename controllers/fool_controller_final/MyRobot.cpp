/**
 * @file    MyRobot.h
 * @brief   Main logic for the fool_controller_final controller.
 *
 * @author  Blake Boyer <100531828@alumnos.uc3m.es>
 * @author  Giordano Arcieri <>
 * @date    2023-12
 */

#include "MyRobot.h"

//////////////// CONSTRUCTER AND DECONSTRUCTER //////////////////////////////

MyRobot::MyRobot() : Robot()
{

    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    target = -90.0;

    calc_x = calc_y = calc_theta = 0.0; // robot pose variables
    _sr = _sl = 0.0;                    // displacement right and left wheels

    vic_count = 0;

    avoiding_obstacle = false;

    navigation_status = TURNING;

    direction = LEFT;

    delay_counter = 2000;

    ////// Motor Initialization //////
    _left_wheel_motor = getMotor("left wheel motor");
    _right_wheel_motor = getMotor("right wheel motor");

    // We set motor position to 0 and then to INFINITY re-initialize the encoder measurement and then allow velocity control
    _right_wheel_motor->setPosition(0.0);
    _left_wheel_motor->setPosition(0.0);
    _right_wheel_motor->setPosition(INFINITY);
    _left_wheel_motor->setPosition(INFINITY);

    // Set motor velocity to 0
    _right_wheel_motor->setVelocity(_left_speed);
    _left_wheel_motor->setVelocity(_right_speed);

    ////// Motor Position Sensor Initialization //////
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);

    ////// Distance Sensors Initialization //////
    for (int i = 0; i < NUMBER_OF_SENSORS; i++)
    {
        std::string name = "ds" + std::to_string(i);
        _distance_sensor[i] = getDistanceSensor(name);
        _distance_sensor[i]->enable(_time_step);
    }

    ////// GPS Initialization //////
    _my_gps = getGPS("gps");
    _my_gps->enable(_time_step);

    ////// Compass Initialization //////
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    ////// Camera Initialization //////
    // Forward Camera
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);

    // Spherical Camera
    _spherical_camera = getCamera("camera_s");
    _spherical_camera->enable(_time_step);
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();

    _my_gps->disable();

    _my_compass->disable();

    // disable distance sensors
    for (int i = 0; i < NUMBER_OF_SENSORS; i++)
    {
        _distance_sensor[i]->disable();
    }

    _forward_camera->disable();
    _spherical_camera->disable();
}

//////////////////////// END ////////////////////////////////////////

///////////////// MAIN DRIVERS /////////////////////////////

void MyRobot::run()
{

    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;

    // updates some attributes and gets width/height of the cameras
    get_camera_size();

    while (step(_time_step) != -1)
    {
        // adjust speed
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);

        cout << "[DATA] : \n";

        // collects all data from robot sensors
        update_robot_data();

        // diplays all robot data from sensors to cout
        display_robot_data();

        cout << "[DRIVERS] : \n";

        if (robot_is_in_endzone() && !victims_found())
        {
            cout << "[MAIN] : Searching Endzone ... \n";
            search_endzone();
        }
        else
        {
            cout << "[MAIN] : Navigating Maze ... \n";
            navigation();
        }

        cout << "******************************************************\n";
    }
}

////////////////////////////////////////

void MyRobot::update_robot_data()
{
    // computed odometry with tics
    compute_odometry();

    // gets distance sensor values
    get_dist_val();

    // gets x y cordinates from GPS
    get_gps_val();

    // updates _theta
    update_global_theta();
}

////////////////////////////////////////

void MyRobot::display_robot_data()
{
    // print speed
    cout << "Speed: L: " << _left_speed << " R: " << _right_speed << endl;

    // print odometry
    print_odometry();

    // display encoder values
    encoder_display();

    //
    distance_sensor_display();

    // display GPS values
    gps_display();

    compass_display();
}

///////////////// NAVIGATION /////////////////////////////

void MyRobot::navigation()
{
    if (!avoiding_obstacle && front_max < 300)
    {
        cout << "[NAV] : No obstacle in front: angle driving\n";
        angle_drive();
    }
    else
    {
        cout << "[NAV] : Obstacle detected: wall following\n";
        wall_follower();
    }
}
//////////////////////////////////////////////

void MyRobot::angle_drive()
{
    if (front_max < 300) 
    {
        if (angle_diff >= -5.0 && angle_diff <= 5)
        {
            cout << "[ANGLE DRIVE] : Moving forward" << endl;
            forward();
        }
        else if (angle_diff < -5 && angle_diff > -40)
        {
            cout << "[ANGLE DRIVE] : Adjusting left" << endl;
            left_turn_adj(); // Small left adjustment
        }
        else if (angle_diff <= -40 && angle_diff >= -180)
        {
            cout << "[ANGLE DRIVE] : Turning left" << endl;
            left_turn(); // Full left turn
        }
        else if (angle_diff > 5 && angle_diff < 40)
        {
            cout << "[ANGLE DRIVE] : Adjusting right" << endl;
            right_turn_adj(); // Small right adjustment
        }
        else if (angle_diff >= 40 && angle_diff <= 180)
        {
            cout << "[ANGLE DRIVE] : Turning right" << endl;
            right_turn(); // Full right turn
        }
        else
        { // This case should handle the scenario where the angle_diff might wrap around beyond 180 degrees
            if (angle_diff < -180)
            {
                cout << "[ANGLE DRIVE] : Extreme angle, turning right" << endl;
                right_turn();
            }
            else if (angle_diff > 180)
            {
                cout << "[ANGLE DRIVE] : Extreme angle, turning left" << endl;
                left_turn();
            }
        }
    }
    else
    {
        avoiding_obstacle = true;
        cout << "[ANGLE DRIVE] : Obstacle too close, initiating obstacle avoidance" << endl;
    }
}

//////////////////////////////////////////////

void MyRobot::wall_follower()
{
    if (avoiding_obstacle == false)
    {
        avoiding_obstacle = true;
        direction = turn_direction();
    }

    // Basic wall following logic
    int sens = 175;
    int ideal_distance = 500;        // Ideal distance from the wall
    int distance_error_margin = 400; // Margin of error for the ideal distance
    if (front_all > 100)
    {
        if (direction == LEFT)
        {
            cout << "L ";
            cout << "[WALL FOLLOWER] : Front obstacle, turning left\n";
            left_turn();
        }
        else
        {
            cout << "R ";
            cout << "[WALL FOLLOWER] : Front obstacle, turning right\n";
            right_turn();
        }
    }
    else if (direction == LEFT)
    {
        cout << "L ";
        if (side_F_R > side_B_R + sens)
        {
            cout << "[WALL FOLLOWER] : " << side_F_R << '|' << side_B_R << " Front much closer than back, turning left to adjust\n";
            left_turn_slow_adj();
        }
        else if (side_B_R > side_F_R + sens)
        {
            cout << "[WALL FOLLOWER] : " << side_F_R << '|' << side_B_R << " Back much closer than front, turning right to adjust\n";
            right_turn_slow_adj();
        }
        else if (side_R < ideal_distance - distance_error_margin)
        {
            cout << "[WALL FOLLOWER] : Too far from the wall, moving closer\n";
            right_turn_slow_adj(); // Move closer to the wall
        }
        else if (side_R > ideal_distance + distance_error_margin)
        {
            cout << "[WALL FOLLOWER] : Too close to the wall, moving away\n";
            left_turn_slow_adj(); // Move away from the wall
        }
        else
        {
            cout << "[WALL FOLLOWER] : " << side_F_R << '|' << side_B_R << " Side sensors are balanced, moving forward\n";
            forward_slow();
        }
    }
    else if (direction == RIGHT)
    {
        cout << "R ";
        if (side_F_L > side_B_L + sens)
        {
            cout << "[WALL FOLLOWER] : " << side_F_L << '|' << side_B_L << " Front much closer than back, turning right to adjust\n";
            right_turn_slow_adj();
        }
        else if (side_B_L > side_F_L + sens)
        {
            cout << "[WALL FOLLOWER] : " << side_F_L << '|' << side_B_L << " Back much closer than front, turning left to adjust\n";
            left_turn_slow_adj();
        }
        else if (side_L < ideal_distance - distance_error_margin)
        {
            cout << "[WALL FOLLOWER] : Too far from the wall, moving closer\n";
            left_turn_slow_adj(); // Move closer to the wall
        }
        else if (side_L > ideal_distance + distance_error_margin)
        {
            cout << "[WALL FOLLOWER] : Too close to the wall, moving away\n";
            right_turn_slow_adj(); // Move away from the wall
        }
        else
        {
            cout << "[WALL FOLLOWER] : " << side_F_L << '|' << side_B_L << " Side sensors are balanced, moving forward\n";
            forward_slow();
        }
    }

    // Check if the robot can stop wall following and go north
    if ((angle_diff >= -10.0 && angle_diff <= 10) && !front)
    {
        avoiding_obstacle = false;
        cout << "[WALL FOLLOWER] : Able to go north, switching to angle drive\n";
    }
}

////////////////////// SERACHING ENDZONE ////////////////////////////

void MyRobot::search_endzone()
{
    cout << "[SEARCH] : Searching Endzone" << endl;
    vic_count = 2;
    target = 90;
}

//////////////////////// END ////////////////////////////////////////

/////////////////// UPDATING FUNCTIONS ////////////////////////////////

void MyRobot::compute_odometry()
{
    float b = WHEELS_DISTANCE;
    float dif_sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue()) - _sl;
    float dif_sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue()) - _sr;

    calc_x = calc_x + (((dif_sr + dif_sl) / 2) * cos(_theta + ((dif_sr - dif_sl) / (2 * b))));
    calc_y = calc_y + (((dif_sr + dif_sl) / 2) * sin(_theta + ((dif_sr - dif_sl) / (2 * b))));
    calc_theta = calc_theta + ((dif_sr - dif_sl) / b);

    _sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue());
    _sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue());
}

//////////////////////////////////////

void MyRobot::get_dist_val()
{
    front_L = _distance_sensor[FRONT_LEFT]->getValue();
    front_R = _distance_sensor[FRONT_RIGHT]->getValue();
    front = (front_L + front_R) / 2.0;

    side_F_L = _distance_sensor[SIDE_FRONT_LEFT]->getValue();
    side_F_R = _distance_sensor[SIDE_FRONT_RIGHT]->getValue();
    side_B_L = _distance_sensor[SIDE_BACK_LEFT]->getValue();
    side_B_R = _distance_sensor[SIDE_BACK_RIGHT]->getValue();
    side_R = (side_F_R + side_B_R) / 2.0;
    side_L = (side_F_L + side_B_L) / 2.0;

    front_M_L = _distance_sensor[FRONT_MIDDLE_LEFT]->getValue();
    front_E_L = _distance_sensor[FRONT_EDGE_LEFT]->getValue();
    edge_L = (front_M_L + front_E_L) / 2.0;

    front_M_R = _distance_sensor[FRONT_MIDDLE_RIGHT]->getValue();
    front_E_R = _distance_sensor[FRONT_EDGE_RIGHT]->getValue();
    edge_R = (front_M_R + front_E_R) / 2.0;

    front_all = (front_L + front_R + front_M_L + front_M_R) / 6.0;
    front_max = max(front_L, max(front_R, max(front_M_L, front_M_R)));
}

//////////////////////////////////////

void MyRobot::get_gps_val()
{
    GPS_X = _my_gps->getValues()[2];
    GPS_Y = _my_gps->getValues()[0];
}

/////////////////////////////////////////////

void MyRobot::update_global_theta()
{
    _theta = convert_bearing_to_degrees();
    angle_diff = target - _theta;
}

//////////////////////////////////////

void MyRobot::get_camera_size()
{
    // get size of images for forward camera
    image_width_f = _forward_camera->getWidth();
    image_height_f = _forward_camera->getHeight();
    // cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // get size of images for spherical camera
    image_width_s = _spherical_camera->getWidth();
    image_height_s = _spherical_camera->getHeight();
    // cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;
}

//////////////////////// END ////////////////////////////////////////

/////////////////// PRINTING FUNCTIONS ////////////////////////////////

void MyRobot::print_odometry()
{
    cout << "Odom : ( x:" << calc_x << " y:" << calc_y << " theta: " << calc_theta << " )" << endl;
}

//////////////////////////////////////

void MyRobot::encoder_display()
{
    cout << "Left encoder: " << _left_wheel_sensor->getValue() << endl;
    cout << "Right encoder: " << _right_wheel_sensor->getValue() << endl;
}

//////////////////////////////////////

void MyRobot::distance_sensor_display()
{
    cout << "F: " << front << ", FA: " << front_all << ", FM: " << front_max << ", L: " << side_L << ", R: " << side_R << ", SFL: " << side_F_L << ", SBL: " << side_B_L << ", SFR: " << side_F_R << ", SBR: " << side_B_R << "\n";
}

//////////////////////////////////////

void MyRobot::gps_display()
{
    cout << "GPS X: " << GPS_X << endl;
    cout << "GPS Y: " << GPS_Y << endl;
}

//////////////////////////////////////

void MyRobot::compass_display()
{
    cout << "Compass: " << _theta << endl;
}

//////////////////////////////////////

void MyRobot::print_found()
{
    if (vic_count == 1)
    {
        // cout << "First victim has been found" << endl;
    }
    else if (vic_count == 2)
    {
        // cout << "Both victims have been located. Returning to start." << endl;
    }
}

//////////////////////// END ////////////////////////////////////////

//////////////////// HELPER FUNCTIONS //////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees()
{
    const double *in_vector = _my_compass->getValues();

    double rad = atan2(in_vector[2], in_vector[0]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////

bool MyRobot::obstacle_detected()
{
    if (front > 0.0 || side_F_L > 0.0 || side_F_R > 0.0 || edge_L > 0.0 || edge_R > 0.0)
    {
        return true;
    }
    return false;
}

//////////////////////////////////////

Direction MyRobot::turn_direction()
{
    static int direction = 0;

    direction += 1;

    return static_cast<Direction>(direction % 2);
}

////////////////////////////////////////

bool MyRobot::robot_is_in_endzone()
{
    return GPS_X >= 9.0;
}

////////////////////////////////////////

bool MyRobot::victims_found()
{
    return vic_count == 2;
}

//////////////////////// END ////////////////////////////////////////

////////////////////////////// MOVING FUNCTIONS ///////////////////////

void MyRobot::left_turn_adj()
{
    _left_speed = MAX_SPEED - 1;
    _right_speed = MAX_SPEED;
}

//////////////////////////////////////////////

void MyRobot::right_turn_adj()
{
    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED - 1;
}

//////////////////////////////////////////////

void MyRobot::right_turn_slow()
{
    _left_speed = SLOW_SPEED;
    _right_speed = -SLOW_SPEED;
}

void MyRobot::right_turn_slow_adj()
{
    _left_speed = 2.5;
    _right_speed = -1.5;
}

void MyRobot::right_turn()
{
    _left_speed = MED_SPEED;
    _right_speed = -MED_SPEED;
}

//////////////////////////////////////////////

void MyRobot::left_turn()
{
    _left_speed = -MED_SPEED;
    _right_speed = MED_SPEED;
}

void MyRobot::left_turn_slow()
{
    _left_speed = -SLOW_SPEED;
    _right_speed = SLOW_SPEED;
}

void MyRobot::left_turn_slow_adj()
{
    _left_speed = -1.5;
    _right_speed = 2.5;
}

//////////////////////////////////////////////

void MyRobot::forward()
{
    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;
}

void MyRobot::forward_slow()
{
    _left_speed = 5;
    _right_speed = 5;
}

//////////////////////////////////////////////

void MyRobot::back()
{
    _left_speed = -MED_SPEED;
    _right_speed = -MED_SPEED;
}

//////////////////////////////////////////////

void MyRobot::approach()
{
    _left_speed = SLOW_SPEED;
    _right_speed = SLOW_SPEED;
}

//////////////////////////////////////////////

void MyRobot::stop()
{
    _left_speed = 0;
    _right_speed = 0;
}

//////////////////////// END ////////////////////////////////////////

void MyRobot::switch_turn()
{
    delay_counter++;
    // cout << "Delay counter: " << delay_counter << endl;
    // cout << "Delay calculation: " << delay_counter % 2000 << endl;

    if (delay_counter % 2000 > 1000)
    {
        turn_option = true;
        // cout << "Turn option is true" << endl;
    }
    else
    {
        turn_option = false;
        // cout << "Turn option is false" << endl;
    }
}
