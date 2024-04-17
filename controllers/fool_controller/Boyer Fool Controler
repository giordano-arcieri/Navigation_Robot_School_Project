/**
 * @file    MyRobot.h
 * @brief   A simple example for computing the odometry while the robot moves straight
 *
 * @author  Juan José Gamboa Montero <jgamboa@ing.uc3m.es>
 * @author  Jesús García Martínez <jesusgar@ing.uc3m.es>
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

    _x = _y = _theta = 0.0; // robot pose variables 
    _sr = _sl = 0.0; // displacement right and left wheels

    _x_goal = 1.55, _y_goal = 0.25, _z_goal = 9.47;
    
    //recieve target angle in degrees relative to starting position
    _theta_goal = (atan2((_y_goal - _y),(_x_goal - _x)) * 180 / M_PI) + 90; 
    
    // Motor Position Sensor initialization
    _left_wheel_sensor = getPositionSensor("left wheel sensor");
    _right_wheel_sensor = getPositionSensor("right wheel sensor");

    _left_wheel_sensor->enable(_time_step);
    _right_wheel_sensor->enable(_time_step);   


    // Get robot's GPS; initialize it  
    _my_gps = getGPS("gps"); 
    _my_gps->enable(_time_step);
    //GPS * _my_gps;
    
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
    cout << "Goal --> Theta: " << _theta_goal << endl;

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
        double front_left = _distance_sensor[0]->getValue();
        double front_right = _distance_sensor[1]->getValue();
        double side_right = _distance_sensor[2]->getValue();
        double side_left = _distance_sensor[3]->getValue();
        
        float _x =_my_gps->getValues()[2]; 
        float _y =_my_gps->getValues()[0];
        
        cout << "Front left: " << front_left << " Front right: " << front_right << 
        " Left side: " << side_left << " Right side: " << side_right << endl;
        
        cout << "Left encoder: " << this->_left_wheel_sensor->getValue() << endl;
        cout << "Right encoder: " << _right_wheel_sensor->getValue() << endl;
         
        cout<< "X: " << _x << endl;
        cout<< "Y: " << _y << endl;
        _theta = convert_bearing_to_degrees();
        
        //green_identifier();
        
        wall_follower(front_right, front_left, side_right, side_left);
       
        _left_wheel_motor->setVelocity(_left_speed);
        _right_wheel_motor->setVelocity(_right_speed);
       
        this->compute_odometry(); 
        this->print_odometry();
        if(this->goal_reached()) break;

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

double MyRobot::convert_bearing_to_degrees()
{
    const double *in_vector = _my_compass->getValues();
    
    double rad = atan2(in_vector[0], in_vector[2]);
    double deg = (rad * (180.0 / M_PI)) + 180;

    return deg;
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

bool MyRobot::goal_reached()
{
  if(_x < _x_goal || _y < _y_goal) {
      return false;
      }
  else {
      return true;
      }
}

//////////////////////////////////////////////

// float MyRobot::angle_drive(float target, float heading)
// {
   // float angle_diff = target - heading;
   
   // cout << "Angle difference: " << angle_diff << endl;
   
   // if(angle_diff < 3.0 && angle_diff > -3.0) {
       // forward();
       // cout << "Angle driving: moving forward" << endl;
       // }
   // else if(angle_diff >= 3 && angle_diff < 180) {
       // right_turn();
       // cout << "Angle driving: turning right" << endl;
       // }
   // else {
       // left_turn();
       // cout << "Angle driving: turning left" << endl;
       // }
   
   // //set the motor speeds
       // _left_wheel_motor->setVelocity(_left_speed);
       // _right_wheel_motor->setVelocity(_right_speed);
   
   // return 0;
// }

///////////////////////////////////////////////

void MyRobot::right_turn()
{
    _left_speed = APPR_TURN_SPEED;
    _right_speed = -APPR_TURN_SPEED;
}

//////////////////////////////////////////////

void MyRobot::left_turn()
{
    _left_speed = -APPR_TURN_SPEED;
    _right_speed = APPR_TURN_SPEED;
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
    _left_speed = -MAX_SPEED;
    _right_speed = -MAX_SPEED;
}

//////////////////////////////////////////////

void MyRobot::approach()
{
    _left_speed = APPR_TURN_SPEED;
    _right_speed = APPR_TURN_SPEED;
}

//////////////////////////////////////////////

void MyRobot::stop()
{
    _left_speed = 0;
    _right_speed = 0;
}
//////////////////////////////////////////////

void MyRobot::wall_follower(double front_R, double front_L, double side_R, double side_L)
{
    if (front_L < 200 && front_R < 200) {
        cout << "Moving forward" << endl;
        forward();
        }
    else if (front_R >= 200) {
        cout << "Turning left" << endl;
        left_turn();
        }
    else if (front_L >= 200) {
        cout << "Turning right" << endl;
        right_turn();
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
