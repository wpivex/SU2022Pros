#include "Subsystems/RobotBuilder.h"
#include "Subsystems/Localizer/Odometry.h"
#include "Subsystems/Localizer/IMULocalizer.h"
#include "Subsystems/Flywheel/TBHFlywheel.h"
#include "Subsystems/Flywheel/BBFFlywheel.h"
#include "pros/motors.h"

Robot getRobot15(bool isSkills) {

    Robot robot;

    robot.drive.reset(new Drive(
        {11, 12, 13, 14}, // left motor ports
        {15, 2, 3, 4}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.73, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    if (isSkills) {
        robot.localizer.reset(new Odometry(
            *robot.drive, // reference to drive object
            8, // imu port A
            9, // imu port B
            5 // gps port
        ));
    }
    else {
        robot.localizer.reset(new IMULocalizer(
            *robot.drive, // reference to drive object
            14, // imu port A
            -1 // imu port B
        ));
    }

    

    robot.flywheel.reset(new TBHFlywheel(
        {1, -2}, // ports
        { // volt to rpm data
            {1615, 5},
            {1966, 6},
            {2306, 7},
            {2646, 8},
            {3054, 9},
            {3416, 10},
            {3751, 11},
            {4141, 12}
        },
        0, // start speed
        0.0002 // tbh constant
    ));

    robot.intake.reset(new pros::MotorGroup({-13, 16}));
    robot.intake->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    robot.indexer.reset(new pros::ADIDigitalOut('G'));

    robot.roller.reset(new pros::Motor(5, pros::E_MOTOR_GEAR_100));
    robot.roller->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    robot.shooterFlap.reset(new pros::ADIDigitalOut('H'));

    robot.endgame.reset(new pros::ADIDigitalOut('B'));

    return robot;

}


Robot getRobot18(bool isSkills) {

    Robot robot;

    robot.drive.reset(new Drive(
        {5,6, -5, 6}, // left motor ports
        {-9, 16, 18, -19}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.74, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    if (isSkills) {
        robot.localizer.reset(new Odometry(
            *robot.drive, // reference to drive object
            8, // imu port A
            9, // imu port B
            5 // gps port
        ));
    }
    else {
        robot.localizer.reset(new IMULocalizer(
            *robot.drive, // reference to drive object
            14, // imu port A
            -1 // imu port B
        ));
    }

    robot.flywheel.reset(new TBHFlywheel(
        {-3, 4}, // ports
        { // volt to rpm data
            {1615, 5},
            {1966, 6},
            {2306, 7},
            {2646, 8},
            {3054, 9},
            {3416, 10},
            {3751, 11},
            {4141, 12}
        },
        0, // start speed
        0.0002 // tbh constant
    ));

    robot.intake.reset(new pros::MotorGroup({11, -12}));
    robot.intake->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    robot.indexer.reset(new pros::ADIDigitalOut('G'));

    robot.roller.reset(new pros::Motor(7, pros::E_MOTOR_GEAR_100));
    robot.roller->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    robot.shooterFlap.reset(new pros::ADIDigitalOut('H'));

    robot.endgame.reset(new pros::ADIDigitalOut('B'));

    return robot;

}