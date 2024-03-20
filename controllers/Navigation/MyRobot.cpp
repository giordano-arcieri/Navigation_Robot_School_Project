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
    right_wheel_motor->setVelocity(0.0);
    left_wheel_motor->setVelocity(0.0);

    // Camera sensors
    forward_camera = getCamera("camera_f");
    forward_camera->enable(time_step);
    spherical_camera = getCamera("camera_s");
    spherical_camera->enable(time_step);

    // Laser sensors
    for(int i = 0; i < 16; i++)
    {
        std::string name = "laser_sensor" + std::to_string(i);
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

    // // GPS
    gps = getGPS("gps");
    gps->enable(time_step);

    
}

MyRobot::~MyRobot()
{
    // This destructor will disable all sensors

    // Camera sensors
    forward_camera->disable();
    spherical_camera->disable();

    // Laser sensors
    // for(int i = 0; i < 16; i++)
    // {
    //     laser_sensors[i]->disable();
    // }

    // Encoders
    left_wheel_encoder->disable();
    right_wheel_encoder->disable();

    // Compass
    compass->disable();

    // GPS
    gps->disable();

}

void MyRobot::run()
{
    // Print the values of all enabled sensors
    std::cout << "Sensor values:" << std::endl;

    // Camera sensors
    // std::cout << "Forward Camera: " << forward_camera->getImage() << std::endl;
    // std::cout << "Spherical Camera: " << spherical_camera->getImage() << std::endl;

    // Laser sensors
    // for(int i = 0; i < 16; i++)
    // {
    //     std::cout << "Laser Sensor " << i << ": " << laser_sensors[i]->getValue() << std::endl;
    // }

    // Encoders
    std::cout << "Left Wheel Encoder: " << left_wheel_encoder->getValue() << std::endl;
    std::cout << "Right Wheel Encoder: " << right_wheel_encoder->getValue() << std::endl;

    // Compass
    std::cout << "Compass: " << compass->getValues()[0] << ", " << compass->getValues()[1] << ", " << compass->getValues()[2] << std::endl;

    // GPS
    std::cout << "GPS: " << gps->getValues()[0] << ", " << gps->getValues()[1] << ", " << gps->getValues()[2] << std::endl;
}
