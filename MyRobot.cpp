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

    target = 90.0;
    
    _x = _y = _theta = 0.0; // robot pose variables 
    _sr = _sl = 0.0; // displacement right and left wheels

    _x_goal = 1.55, _y_goal = 0.25, _z_goal = 9.47;
    
    //recieve target angle in degrees relative to starting position
    //_theta_goal = (atan2((_y_goal - _y),(_x_goal - _x)) * 180 / M_PI) + 90; 
    _theta_goal = 90.0;
    
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

    // Set motor position to 0 to re-initialize the encoder measurement
    _right_wheel_motor->setPosition(0.0);
    _left_wheel_motor->setPosition(0.0); 

    // Set motor position to infinity to allow velocity control
    _right_wheel_motor->setPosition(INFINITY); 
    _left_wheel_motor->setPosition(INFINITY);

    // Set motor velocity to 0
    _right_wheel_motor->setVelocity(0.0);
    _left_wheel_motor->setVelocity(0.0);

    // get distance sensor array and enable each one 
    
    //Ds1 is front left
    _distance_sensor[0] = getDistanceSensor("ds1"); 
    _distance_sensor[0]->enable(_time_step); 
    
    //Ds14 is front right
    _distance_sensor[1] = getDistanceSensor("ds14"); 
    _distance_sensor[1]->enable(_time_step); 
    
    //Ds12 is far right sensor
    _distance_sensor[2] = getDistanceSensor("ds12"); 
    _distance_sensor[2]->enable(_time_step);
    
    //Ds3 is far left sensor
    _distance_sensor[3] = getDistanceSensor("ds3"); 
    _distance_sensor[3]->enable(_time_step);
    
    // get cameras and enable them
    _forward_camera = getCamera("camera_f");
    _forward_camera->enable(_time_step);
    //_spherical_camera = getCamera("camera_s");
    //_spherical_camera->enable(_time_step);
    
    
}

//////////////////////////////////////////////

MyRobot::~MyRobot()
{
    // Stop motors
    _left_wheel_motor->setVelocity(0.0);
    _right_wheel_motor->setVelocity(0.0);
    
    // Disable robot's sensors
    _my_compass->disable();
    _left_wheel_sensor->disable();
    _right_wheel_sensor->disable();
    
    // disable camera devices
    _forward_camera->disable();
    //_spherical_camera->disable();
}

//////////////////////////////////////////////
// Controller main logic 
void MyRobot::run()
{
    cout << "Goal --> x: " << _x_goal << endl;
    cout << "Goal --> y: " << _y_goal << endl;
    cout << "Goal --> Theta: " << target << endl;

    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;

    // set the motor speeds
    _left_wheel_motor->setVelocity(_left_speed);
    _right_wheel_motor->setVelocity(_right_speed);
    
    //int green_count_L = 0;
    //int green_count_R = 0;
    
    // unsigned char green = 0, red = 0, blue = 0;
    // double percentage_white_L = 0.0;
    // double percentage_white_R = 0.0;
    
    // get size of images for forward camera
    int image_width_f = _forward_camera->getWidth();
    int image_height_f = _forward_camera->getHeight();
    cout << "Size of forward camera image: " << image_width_f << ", " <<  image_height_f << endl;

    // get size of images for spherical camera
    //int image_width_s = _spherical_camera->getWidth();
    //int image_height_s = _spherical_camera->getHeight();
    //cout << "Size of spherical camera image: " << image_width_s << ", " << image_height_s << endl;
    
    while (step(_time_step) != -1) 
    {
        get_gps_val();
        
        get_dist_val();
        
        dist_display();
        
        encoder_display();
        
        gps_display();
        
        convert_bearing_to_degrees();
        
        switch_drive();
        
        if(obstacle_detected) {    
            wall_follower();
            }
        else { 
            angle_drive();
            }
        
        set_velo();
        
        this->compute_odometry(); 
        this->print_odometry();
       

    }
}

//////////////////////////////////////////////
void MyRobot::compute_odometry()
{
  float b = WHEELS_DISTANCE;
  float dif_sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue()) - _sl;
  float dif_sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue()) - _sr;
  
  _x = _x + (((dif_sr + dif_sl) / 2) * cos(_theta + ((dif_sr - dif_sl) / (2 * b))));
  _y = _y + (((dif_sr + dif_sl) / 2) * sin(_theta + ((dif_sr - dif_sl) / (2 * b))));
  
  _sl = encoder_tics_to_meters(this->_left_wheel_sensor->getValue());
  _sr = encoder_tics_to_meters(this->_right_wheel_sensor->getValue());
  
}        
//////////////////////////////////////////////

void MyRobot::convert_bearing_to_degrees()
{
    const double *in_vector = _my_compass->getValues();
    
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = rad * (180.0 / M_PI);

    _theta = deg;
}
//////////////////////////////////////////////

float MyRobot::encoder_tics_to_meters(float tics)
{
    return tics/ENCODER_TICS_PER_RADIAN * WHEEL_RADIUS;
}

//////////////////////////////////////////////

void MyRobot::print_odometry()
{
    cout << "x:" << _x << " y:" << _y << " theta:" << _theta << endl;
}

//////////////////////////////////////////////

void MyRobot::angle_drive()
{
    float angle_diff = target - _theta;
   
    cout << "Angle difference: " << angle_diff << endl;
    
    if (angle_diff >= -5 && angle_diff <= 5) {
        cout << "Moving forward" << endl;
        forward();
    }
    else if (angle_diff > 5 && angle_diff < 180) {
        cout << "Turning left" << endl;
        left_turn_adj();
    }
    else { // Handles angle_diff < -5 || angle_diff > 180
        cout << "Turning right" << endl;
        right_turn_adj();
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
}

//////////////////////////////////////////////

void MyRobot::left_turn_adj()
{
    _left_speed = MED_SPEED;
    _right_speed = MAX_SPEED;
}

//////////////////////////////////////////////

void MyRobot::right_turn()
{
    _left_speed = MAX_SPEED;
    _right_speed = -MAX_SPEED;
}

//////////////////////////////////////////////

void MyRobot::left_turn()
{
    _left_speed = -MAX_SPEED;
    _right_speed = MAX_SPEED;
}

//////////////////////////////////////////////

void MyRobot::forward()
{
    _left_speed = MAX_SPEED;
    _right_speed = MAX_SPEED;
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
//////////////////////////////////////////////

void MyRobot::wall_follower()
{
    //Two other cases still need to code
    //The front sensor and corresponsing side sensor reading >0 triggers the approach speed
    //it slows down the robot too much and takes forever to correct
    
    //The robot should respond to an obstacle directly in front of it more intelligently
       //Could use the angle mixed with which side on the front is closer to decide which way to turn
     
//Instances
    //Wall in front of the robot -> turn right
    if (front_left > 200 || front_right > 200) {
        cout << "Wall in front; turning left" << endl;
        right_turn();
        }
    //Obstacle detected -> approach slowly
    else if(front_left > 0 || front_right > 0) {
        cout << "Obstacle detected; slowing down" << endl;
        approach();
        }
    //Wall to the left of the robot & no wall in front -> drive straight
    else if (side_left > 200 && front_left == 0 && front_right == 0) {
        cout << "Wall to the left and no wall in front; driving straight" << endl;
        forward();
        }
    //Wall to the right of the robot & no wall in front - > drive straight
    else if (side_right > 200 && front_left == 0 && front_right == 0) {
        cout << "Wall to the right and no wall in front; driving straight" << endl;
        forward();
        }
    //Wall in front of the robot & wall to the right -> turn left
    else if (front_left > 200 && front_right > 200 && side_right) {
        cout << "Wall to the right and wall in front; turning left" << endl;
        left_turn();
        }
    //Wall in front of the robot & wall to the left -> turn right
    else if (front_left > 200 && front_right > 200 && side_left) {
        cout << "Wall to the left and wall in front; turning right" << endl;
        right_turn();
        }
//Special cases
    //Wall to the left gets too close -> turn slightly right
    else if (front_left == 0 && front_right == 0 && side_left > 500) {
        cout << "Left wall too close; adjusting right" << endl;
        right_turn_adj();
        }
    //Wall to the right gets too close -> turn slightly left
    else if (front_left == 0 && front_right == 0 && side_right > 500) {
        cout << "Right wall too close; adjusting left" << endl;
        left_turn_adj();
        }
}

//////////////////////////////////////////////

// void MyRobot::green_identifier()
// {
    // green_count_L = 0;
    // green_count_R = 0;
        
   //// get current image from forward camera
    // const unsigned char *image_f = _forward_camera->getImage();
  ////   const unsigned char *image_s = _spherical_camera->getImage();
        
   //// count number of pixels that are white on left side
   //// (here assumed to have pixel value > 245 out of 255 for all color components)
    // for (int x = 0; x < image_width_f / 2; x++) {
        // for (int y = 0; y < image_height_f; y++) {
            // green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            // red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            // blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            // if ((green > GREEN_THRESHOLD) && (red < GREEN_THRESHOLD) && (blue < GREEN_THRESHOLD)) {
                    // green_count_L = green_count_L + 1;
            // }
        // }
    // }
        
  ////  count number of pixels that are white on right side
    // for (int x = image_width_f / 2; x < image_width_f; x++) {
        // for (int y = 0; y < image_height_f; y++) {
            // green = _forward_camera->imageGetGreen(image_f, image_width_f, x, y);
            // red = _forward_camera->imageGetRed(image_f, image_width_f, x, y);
            // blue = _forward_camera->imageGetBlue(image_f, image_width_f, x, y);

            // if ((green > WHITE_THRESHOLD) && (red < WHITE_THRESHOLD) && (blue < WHITE_THRESHOLD)) {
                    // green_count_R = green_count_R + 1;
            // }
        // }
    // }
        
        
    // cout << "Number of white pixels left: " << white_count_L << endl;
        
    // percentage_green_L = (green_count_L / (float) (image_width_f * image_height_f)) * 100;
    // cout << "Percentage of green in forward left camera image: " << percentage_green_L << endl;
        
    // percentage_green_R = (green_count_R / (float) (image_width_f * image_height_f)) * 100;
    // cout << "Percentage of green in forward right camera image: " << percentage_green_R << endl;
        
//}

//////////////////////////////////////

void MyRobot::get_gps_val()
{
    _x =_my_gps->getValues()[2]; 
    _y =_my_gps->getValues()[0];
}

//////////////////////////////////////

void MyRobot::get_dist_val()
{
    front_left = _distance_sensor[0]->getValue();
    front_right = _distance_sensor[1]->getValue();
    side_right = _distance_sensor[2]->getValue();
    side_left = _distance_sensor[3]->getValue();
}

//////////////////////////////////////

void MyRobot::dist_display()
{
    cout << "Front left: " << front_left << " Front right: " << front_right << 
    " Left side: " << side_left << " Right side: " << side_right << endl;
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
    cout<< "X: " << _x << endl;
    cout<< "Y: " << _y << endl;
}

//////////////////////////////////////

void MyRobot::switch_drive()
{
    if(front_left > 0 || front_right > 0)
    {
        obstacle_detected = true;
    }
    else {
        obstacle_detected = false;
    }
}


//include a condition about the direction of the robot
//continue to wall follow until it reaches the right angle
//alterante between right and left -->try this first to make the program at least functional
//or record the relative coordinates where the robot is getting stuck
  //the origin is just 0,0
  //store the place
  
  
  //if the two sensors facing the wall are relatively similar then drive forward
  //800 maybe use speed 0.1
  //make the odometry work
  //Array or hash table for a vector???
  //struct
  
///////////////////////////////////////

//Searching program

  //Easiest situation
  //Both cylinders on ends of endzone
  //Distinguishing between cylinders close to each other
  //Save the relative position and make sure that it is different by a significant
      //enough margin when the following objects are detected

///////////////////////////////////////

//Turn 360 program

///////////////////////////////////////

//Identify if the robot is in the "endzone" with GPS and change Boolean var

///////////////////////////////////////

//Count number of people found

///////////////////////////////////////

//Distance traveled calculation

////////////////////////////////////////

//Clock get time function for limiting time searching
//Some libraries only work in linux
//Maybe download linux
//Make sure funcitons work in both OS
