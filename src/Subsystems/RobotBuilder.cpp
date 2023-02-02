#include "Subsystems/RobotBuilder.h"
#include "Subsystems/Localizer/Odometry.h"
#include "Subsystems/Flywheel/TBHFlywheel.h"
#include "pros/motors.h"

Robot getRobot15() {

    Robot robot;

    robot.drive.reset(new Drive(
        {-11, 12, -14, 15}, // left motor ports
        {17, 18, -19, -20}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.74, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    robot.localizer.reset(new Odometry(
        8, // imu port
        {'E','F'}, // left encoder port
        {'C', 'D'}, // right encoder port
        {'A', 'B'}, // back encoder port
        1.625, // odom wheel diameter in inches
        5.5 // distance from center of rotation to back encoder in inches
    ));

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

    robot.roller.reset(new pros::Motor(10, pros::E_MOTOR_GEAR_100));
    robot.roller->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    robot.shooterFlap.reset(new pros::ADIDigitalOut('H'));

    robot.endgame.reset(new pros::ADIDigitalOut('B'));

    return robot;

}


Robot getRobot18() {

    Robot robot;

    robot.drive.reset(new Drive(
        {-3, 4, -5, 6}, // left motor ports
        {-9, 16, 18, -19}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.74, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    robot.localizer.reset(new Odometry(
        14, // imu port
        {'E','F'}, // left encoder port
        {'C', 'D'}, // right encoder port
        {'A', 'B'}, // back encoder port
        1.625, // odom wheel diameter in inches
        5.5 // distance from center of rotation to back encoder in inches
    ));

    robot.flywheel.reset(new TBHFlywheel(
        {-1, 2}, // ports
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