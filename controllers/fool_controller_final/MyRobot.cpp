/**
 * @file    MyRobot.h
 * @brief   Main logic for the fool_controller_final controller.
 *
 * @author  Blake Boyer <100531828@alumnos.uc3m.es>
 * @author  Giordano Arcieri <>
 * @date    2023-12
 */

#include "MyRobot.h"

//////////////////////////////////////////////

MyRobot::MyRobot() : Robot()
{

    // init default values
    _time_step = 64;

    _left_speed = 0;
    _right_speed = 0;

    target = -90.0;

    calc_x = calc_y = calc_theta = 0.0; // robot pose variables
    _sr = _sl = 0.0;                    // displacement right and left wheels

    count_turn = 1;

    tick = true;

    cross_count = 1;

    vic_count = 0;

    delay_counter = 2000;

    has_turned = true;

    // Motor Position Sensor initialization
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");
    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);

    // Get robot's GPS; initialize it
    _my_gps = getGPS("gps");
    _my_gps->enable(_time_step);

    // Get robot's compass; initialize it
    _my_compass = getCompass("compass");
    _my_compass->enable(_time_step);

    // Motor initialization
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

    // get distance sensor array and enable each one

    // Ds0 is front left
    _distance_sensor[0] = getDistanceSensor("ds0");
    _distance_sensor[0]->enable(_time_step);

    // Ds15 is front right
    _distance_sensor[1] = getDistanceSensor("ds15");
    _distance_sensor[1]->enable(_time_step);

    // Ds3 is on the side; forward left
    _distance_sensor[2] = getDistanceSensor("ds3");
    _distance_sensor[2]->enable(_time_step);

    // Ds12 is on the side; forward right
    _distance_sensor[3] = getDistanceSensor("ds12");
    _distance_sensor[3]->enable(_time_step);

    // Ds4 is on the side; back left
    _distance_sensor[4] = getDistanceSensor("ds4");
    _distance_sensor[4]->enable(_time_step);

    // Ds4 is on the side; back right
    _distance_sensor[5] = getDistanceSensor("ds11");
    _distance_sensor[5]->enable(_time_step);

    // Ds1 is on the front; left middle sensor
    _distance_sensor[6] = getDistanceSensor("ds1");
    _distance_sensor[6]->enable(_time_step);

    // Ds2 is on the front; left edge sensor
    _distance_sensor[7] = getDistanceSensor("ds2");
    _distance_sensor[7]->enable(_time_step);

    // Ds14 is on the front; right middle sensor
    _distance_sensor[8] = getDistanceSensor("ds14");
    _distance_sensor[8]->enable(_time_step);

    // Ds13 is on the front; right edge sensor
    _distance_sensor[9] = getDistanceSensor("ds13");
    _distance_sensor[9]->enable(_time_step);

    // get cameras and enable them
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);

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
    _distance_sensor[0]->disable();
    _distance_sensor[1]->disable();
    _distance_sensor[2]->disable();
    _distance_sensor[3]->disable();
    _distance_sensor[4]->disable();
    _distance_sensor[5]->disable();
    _distance_sensor[6]->disable();
    _distance_sensor[7]->disable();
    _distance_sensor[8]->disable();
    _distance_sensor[9]->disable();

    _forward_camera->disable();
    _spherical_camera->disable();
}

//////////////////////////////////////////////
// Controller main logic
void MyRobot::run()
{

    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;

    // set the motor speeds
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);

    while (step(_time_step) != -1)
    {

        // updates some attributes and gets width/height of the cameras
        get_camera_size();

        // gets x y cordinates from GPS
        get_gps_val();

        // gets distance sensor values
        get_dist_val();

        // checks how many yellow pixels. if % > 1 and GPS_X == 9, increment cross_count
        crossed_endline();

        // updates bool endzone if GPS_X == 9
        in_endzone();

        // Dispays encoder values to cout
        encoder_display();

        // Dispays gps values to cout
        gps_display();

        // updates _theta
        update_global_theta();

        // computed odometry with tics
        compute_odometry();

        // displays odom
        print_odometry();

        // updates obstacle_detected if any walls detected
        switch_drive();

        // updates turn option randomly
        switch_turn();

        // if in endzone and less than 2 victims found, search for endzone, else if victim found navigate back to start else navigate to endzone
        state_change();

        // green_identifier();

        // search_endzone();
    }
}

//////////////////////////////////////////////

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

//////////////////////////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics / ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

//////////////////////////////////////////////

void MyRobot::print_odometry()
{
    cout << "x:" << calc_x << " y:" << calc_y << " theta: " << calc_theta << endl;
}

//////////////////////////////////////////////

double MyRobot::convert_bearing_to_degrees()
{
    const double *in_vector = _my_compass->getValues();

    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    return deg;
}

//////////////////////////////////////////////

void MyRobot::update_global_theta()
{
    _theta = convert_bearing_to_degrees();
}

//////////////////////////////////////////////

void MyRobot::angle_drive()
{
    angle_diff = target - _theta;

    cout << "Desired angle: " << target << " Angle difference: " << angle_diff << endl;
    if (target < 0.0)
    {
        if (angle_diff >= -1.0 && angle_diff <= 1)
        {
            cout << "Moving forward" << endl;
            forward();
        }
        else if (angle_diff < -1 && angle_diff >= -180)
        {
            cout << "Turning left" << endl;
            left_turn_adj();
        }
        else
        { // Handles angle_diff < -5 || angle_diff > 180
            cout << "Turning right" << endl;
            right_turn_adj();
        }
    }
    else
    {
        if (1)
        {
        }
        else if (1)
        {
        }
        else
        {
        }
    }
}

///////////////////////////////////////////////

void MyRobot::set_velo()
{
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
}

//////////////////////////////////////////////

void MyRobot::right_turn_adj()
{
    _left_speed = MAX_SPEED;
    _right_speed = MED_SPEED;
    set_velo();
}

//////////////////////////////////////////////

void MyRobot::left_turn_adj()
{
    _left_speed = MED_SPEED;
    _right_speed = MAX_SPEED;
    set_velo();
}

//////////////////////////////////////////////

void MyRobot::right_turn()
{
    _left_speed = SLOW_SPEED;
    _right_speed = -SLOW_SPEED;
    set_velo();
}

//////////////////////////////////////////////

void MyRobot::left_turn()
{
    _left_speed = -SLOW_SPEED;
    _right_speed = SLOW_SPEED;
    set_velo();
}

//////////////////////////////////////////////

void MyRobot::forward()
{
    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;
    set_velo();
}

//////////////////////////////////////////////

void MyRobot::back()
{
    _left_speed = -MED_SPEED;
    _right_speed = -MED_SPEED;
    set_velo();
}

//////////////////////////////////////////////

void MyRobot::approach()
{
    _left_speed = SLOW_SPEED;
    _right_speed = SLOW_SPEED;
    set_velo();
}

//////////////////////////////////////////////

void MyRobot::stop()
{
    _left_speed = 0;
    _right_speed = 0;
    set_velo();
}
//////////////////////////////////////////////

void MyRobot::state_change()
{
    if (endzone && vic_count < 2)
    {
        search_endzone();
        // cout << "Searching endzone. Number of victims found: " << vic_count << endl;
    }
    else if (vic_count >= 2)
    {
        target = 90.0;
        navigation();
        // cout << "Both victims found. Navigating back to starting point" << endl;
    }
    else
    {
        navigation();
        // cout << "Navigating to endzone" << endl;
    }
}
//////////////////////////////////////////////

void MyRobot::navigation()
{
    if (obstacle_detected)
    {
        wall_follower();
        cout << "Obstacle detected: wall following" << endl;
    }
    else
    {
        angle_drive();
        cout << "No obstacle in front: angle driving" << endl;
    }
}
//////////////////////////////////////////////

void MyRobot::get_dist_val()
{
    front_L = _distance_sensor[0]->getValue();
    front_R = _distance_sensor[1]->getValue();
    front = (front_L + front_R) / 2.0;

    side_F_L = _distance_sensor[2]->getValue();
    side_F_R = _distance_sensor[3]->getValue();
    side_B_L = _distance_sensor[4]->getValue();
    side_B_R = _distance_sensor[5]->getValue();

    front_M_L = _distance_sensor[6]->getValue();
    front_E_L = _distance_sensor[7]->getValue();
    edge_L = (front_M_L + front_E_L) / 2.0;

    front_M_R = _distance_sensor[8]->getValue();
    front_E_R = _distance_sensor[9]->getValue();
    edge_R = (front_M_R + front_E_R) / 2.0;

    // cout << "***Front sensor display***" << endl;
    // cout << "Front: " << front << endl;
    // cout << "Front edge right: " << edge_R << " Front side right: " << side_F_R << endl;
    // cout << "Front edge left: " << edge_L << " Front side left: " << side_F_L << endl;
    // cout << "***End distance display***" << endl;
}

//////////////////////////////////////

void MyRobot::wall_follower()
{
    angle_diff = target - _theta;

    // cout << "Angle difference is: " << _theta << endl;

    if (front > 0.0 && (side_F_R > 0.0 || edge_R > 0.0))
    {
        cout << "Case 1" << endl;
        stop();
        left_turn();
    }
    else if (front > 0.0 && (side_F_L > 0.0 || edge_L > 0.0))
    {
        cout << "Case 2" << endl;
        stop();
        right_turn();
    }
    else if (side_F_L > 200.0 || edge_L > 0.0)
    {
        cout << "Case 3" << endl;
        right_turn_adj();
    }
    else if (side_F_R > 200.0 || edge_R > 0.0)
    {
        cout << "Case 4" << endl;
        left_turn_adj();
    }
    // Side front left case
    else if (side_F_L > 0.0)
    {
        right_turn();
        stop();
        if ((side_F_L > side_B_L + 200.0) || (side_F_L < side_B_L - 200.0))
        {
            right_turn();
            cout << "Case 5" << endl;
        }
        else
        {
            forward();
            cout << "Case 6" << endl;
        }
    }

    // Side front right case
    else if (side_F_R > 0.0)
    {
        left_turn();
        stop();
        if ((side_F_R > side_B_R + 200.0) || (side_F_R < side_B_R - 200.0))
        {
            left_turn();
            cout << "Case 7" << endl;
        }
        else
        {
            forward();
            cout << "Case 8" << endl;
        }
    }
    else if (front > 0.0 && side_F_L == 0.0 && side_F_R == 0.0)
    {
        stop();
        if (turn_option)
        {
            cout << "Case 9" << endl;
            right_turn();
        }
        else
        {
            cout << "Case 10" << endl;
            left_turn();
        }
    }
    else
    {
        forward();
    }
}

//////////////////////////////////////////////

void MyRobot::green_identifier()
{
    int green_count_L = 0;
    int green_count_R = 0;

    // get current image from forward camera
    const unsigned char *image_f = _forward_camera->getImage();

    // count number of pixels that are green on the left half of the camera
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (int x = 0; x < image_width_f / 2; x++)
    {
        for (int y = 0; y < image_height_f; y++)
        {
            green_L = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            red_L = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            blue_L = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            if ((green_L > GREEN_THRESHOLD) && (red_L < GREEN_THRESHOLD) && (blue_L < GREEN_THRESHOLD))
            {
                green_count_L = green_count_L + 1;
            }
        }
    }

    // count number of pixels that are green on the right half of the camera
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (int a = image_width_f / 2; a < image_width_f; a++)
    {
        for (int b = 0; b < image_height_f; b++)
        {
            green_R = _forward_camera->imageGetGreen(image_f, image_width_f, a, b);
            red_R = _forward_camera->imageGetRed(image_f, image_width_f, a, b);
            blue_R = _forward_camera->imageGetBlue(image_f, image_width_f, a, b);

            if ((green_R > GREEN_THRESHOLD) && (red_R < GREEN_THRESHOLD) && (blue_R < GREEN_THRESHOLD))
            {
                green_count_R = green_count_R + 1;
            }
        }
    }

    // cout << "Number of green pixels on left half of camera: " << green_count_L << endl;
    // cout << "Number of green pixels on right half of camera: " << green_count_R << endl;

    percentage_green_L = (green_count_L / (float)((image_width_f / 2) * image_height_f)) * 100;
    // cout << "Percentage of green pixels in left half of camera: " << percentage_green_L << endl;

    percentage_green_R = (green_count_R / (float)((image_width_f / 2) * image_height_f)) * 100;
    // cout << "Percentage of green pixels in right half of camera: " << percentage_green_R << endl;
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
//////////////////////////////////////

void MyRobot::get_gps_val()
{
    GPS_X = _my_gps->getValues()[2];
    GPS_Y = _my_gps->getValues()[0];
}

//////////////////////////////////////

void MyRobot::encoder_display()
{
    cout << "Left encoder: " << this->_left_wheel_sensor->getValue() << endl;
    cout << "Right encoder: " << _right_wheel_sensor->getValue() << endl;
}

//////////////////////////////////////

void MyRobot::gps_display()
{
    cout << "GPS X: " << GPS_X << endl;
    cout << "GPS Y: " << GPS_Y << endl;
}

//////////////////////////////////////

void MyRobot::in_endzone()
{
    // crossed_endline();

    if (GPS_X == 9)
    {
        endzone = true;
        // cout << "In endzone" << endl;
    }
    else
    {
        endzone = false;
        // cout << "Not in endzone" << endl;
    }
}

//////////////////////////////////////

void MyRobot::crossed_endline()
{
    int yellow_count = 0;

    // get current image from forward camera
    const unsigned char *image_s = _spherical_camera->getImage();

    // count number of pixels that are white on left side
    // (here assumed to have pixel value > 245 out of 255 for all color components)
    for (int i = 0; i < image_width_s; i++)
    {
        for (int j = 0; j < image_height_s; j++)
        {
            green_S = _spherical_camera->imageGetGreen(image_s, image_width_s, i, j);
            red_S = _spherical_camera->imageGetRed(image_s, image_width_s, i, j);
            blue_S = _spherical_camera->imageGetBlue(image_s, image_width_s, i, j);

            if ((green_S > YELLOW_THRESHOLD) && (red_S > YELLOW_THRESHOLD) && (blue_S < YELLOW_THRESHOLD))
            {
                yellow_count = yellow_count + 1;
            }
        }
    }

    // cout << "Yellow count test: " << yellow_count << endl;

    percentage_yellow = (yellow_count / (float)(image_width_s * image_height_s)) * 100;
    // cout << "Percentage of yellow: " << percentage_yellow << " GPS Value: " << GPS_X <<endl;

    if (percentage_yellow > 1 && GPS_X == 9)
    {
        cross_count++;
    }

    // cout << "Cross count test: " << cross_count << endl;
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

//////////////////////////////////////

void MyRobot::search_endzone()
{
    green_identifier();

    // no person has been found
    if (vic_count == 0)
    {
        green_drive();
    }

    // one person has been found
    else if (vic_count == 1)
    {
        right_turn();
        int vic_heading_diff = _theta - vic_heading;
        // cout << "Heading difference is: " << vic_heading_diff << endl;

        if (vic_heading_diff < 50 && vic_heading_diff > -50)
        {
            right_turn();
        }
        else
        {
            green_drive();
        }
    }
    // both people have been found
    // end search
    else if (vic_count == 2)
    {
        stop();
    }
}
//////////////////////////////////////

void MyRobot::green_drive()
{
    // if neither side of camera detects green, turn right
    if (percentage_green_L == 0 && percentage_green_R == 0)
    {
        right_turn();
        // cout << "No green; turning right" << endl;
    }

    // if both sides detect same amount of green, greater than 30 percent, stop
    else if (percentage_green_L == 100 && percentage_green_R == 100)
    {
        vic_count++;
        print_found();
        vic_heading = _theta;
        stop();
    }

    // if both detect same amount of green and they are less than 30 percent, forward
    // tolerance 3 percent
    else if (percentage_green_L < (percentage_green_R + GREEN_BUFFER) && percentage_green_L > (percentage_green_R - GREEN_BUFFER))
    {
        forward();
        // cout << "Green in front; forward" << endl;
    }

    // if right side of camera detects more green than the left side, turn right
    // tolerance 5 percent
    else if (percentage_green_R > percentage_green_L)
    {
        right_turn_adj();
        // cout << "Green on right; turning right" << endl;
    }

    // if left side of camera detects more green than the right side, turn left
    // tolerance 5 percent
    else if (percentage_green_R < percentage_green_L)
    {
        left_turn_adj();
        // cout << "Green on left; turning left" << endl;
    }
}
//////////////////////////////////////

void MyRobot::switch_drive()
{
    if (front > 0.0 || side_F_L > 0.0 || side_F_R > 0.0 || edge_L > 0.0 || edge_R > 0.0)
    {
        obstacle_detected = true;
    }
    else
    {
        obstacle_detected = false;
    }
}

//////////////////////////////////////

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

////////////////////////////////////////
