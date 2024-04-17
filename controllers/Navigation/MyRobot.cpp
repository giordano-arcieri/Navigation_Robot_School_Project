#include "MyRobot.h"

MyRobot::MyRobot()
{
    // This Constructer will init all default values and enable all sensors

    // Time step
    this->time_step = 64;

    // Velocities
    this->left_speed = 0;
    this->right_speed = 0;

    // Motors
    left_wheel_motor = getMotor("left wheel motor");
    right_wheel_motor = getMotor("right wheel motor");

    // We set motor position to 0 and then to INFINITY re-initialize the encoder measurement and then allow velocity control
    right_wheel_motor->setPosition(0.0);
    left_wheel_motor->setPosition(0.0);
    right_wheel_motor->setPosition(INFINITY);
    left_wheel_motor->setPosition(INFINITY);

    // Set motor velocity to 0
    right_wheel_motor->setVelocity(this->right_speed);
    left_wheel_motor->setVelocity(this->left_speed);

    // Camera sensors
    forward_camera = getCamera("camera_f");
    forward_camera->enable(time_step);
    spherical_camera = getCamera("camera_s");
    spherical_camera->enable(time_step);

    // Laser sensors
    for (int i = 0; i < NUMBER_OF_SENSORS; i++)
    {
        std::string name = "ds" + std::to_string(i);
        laser_sensors[i] = getDistanceSensor(name);
        laser_sensors[i]->enable(time_step);
    }

    // Encoders
    left_wheel_encoder = getPositionSensor("left wheel sensor");
    right_wheel_encoder = getPositionSensor("right wheel sensor");
    left_wheel_encoder->enable(time_step);
    right_wheel_encoder->enable(time_step);

    // Compass
    compass = getCompass("compass");
    compass->enable(time_step);

    // GPS
    gps = getGPS("gps");
    gps->enable(time_step);

    // Log the construction of the robot
    Logger::log("Robot class constructed");
}

MyRobot::~MyRobot()
{
    // This destructor will disable all sensors

    // Camera sensors
    forward_camera->disable();
    spherical_camera->disable();

    // Laser sensors
    for (int i = 0; i < NUMBER_OF_SENSORS; i++)
    {
        laser_sensors[i]->disable();
    }

    // Encoders
    left_wheel_encoder->disable();
    right_wheel_encoder->disable();

    // Compass
    compass->disable();

    // GPS
    gps->disable();

    // Log the destruction of the robot
    Logger::log("Robot class destroyed");
}

void MyRobot::run()
{
    // This function will be called every time step

    // Log the run of the robot
    Logger::log("Robot class run");
    
}

void MyRobot::set_left_speed(double speed)
{
    this->left_speed = speed;
    left_wheel_motor->setVelocity(this->left_speed);

    Logger::log("Left wheel speed set to " + std::to_string(this->left_speed));
}

void MyRobot::set_right_speed(double speed)
{
    this->right_speed = speed;
    right_wheel_motor->setVelocity(this->right_speed);

    Logger::log("Right wheel speed set to " + std::to_string(this->right_speed));
}

double MyRobot::get_laser_index_value(int index) const
{
    return laser_sensors[index]->getValue();
}

std::array<double, NUMBER_OF_SENSORS> MyRobot::get_laser_sensors_values() const
{
    std::array<double, NUMBER_OF_SENSORS> laser_values;
    for (int i = 0; i < NUMBER_OF_SENSORS; i++)
    {
        laser_values[i] = laser_sensors[i]->getValue();
    }
    return laser_values;
}

double MyRobot::get_left_wheel_encoder_value() const
{
    return left_wheel_encoder->getValue();
}

double MyRobot::get_right_wheel_encoder_value() const
{
    return right_wheel_encoder->getValue();
}

std::array<double, 3> MyRobot::get_compass_value() const
{
    return {compass->getValues()[0], compass->getValues()[1], compass->getValues()[2]};
}

std::array<double, 3> MyRobot::get_gps_value() const
{
    return {gps->getValues()[0], gps->getValues()[1], gps->getValues()[2]};
}