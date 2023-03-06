#include "Subsystems/RobotBuilder.h"
#include "Subsystems/Localizer/Odometry.h"
#include "Subsystems/Localizer/IMULocalizer.h"
#include "Subsystems/Flywheel/TBHFlywheel.h"
#include "pros/motors.h"

Robot getRobot15(bool isSkills) {

    Robot robot;

    robot.drive.reset(new Drive(
        {-13, 14, 15, 16}, // left motor ports
        {-17, 18, -19, 20}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.73, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    robot.localizer.reset(new IMULocalizer(
        *robot.drive, // reference to drive object
        1,// relabled to 1, 2, // imu port A
        8 // imu port B
    ));


    

    robot.flywheel.reset(new TBHFlywheel(
        {9, -4}, // ports
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
        { // rpm to distance data FOR FLAP DOWN
            {2450, 56},
            {2425, 61},
            {2475, 66},
            {2550, 71},
            {2575, 76},
            {2700, 81},
            {2800, 86},
            {2800, 91},
            {2850, 96},
            {2925, 101},
            {3050, 106},
            {3187, 111},
            {3225, 116},
            {3350, 121}
            
            
        },
        { // rpm to distance data FOR FLAP UP
            {2450, 56},
            {2425, 61},
            {2475, 66},
            {2550, 71},
            {2575, 76},
            {2700, 81},
            {2800, 86},
            {2800, 91},
            {2850, 96},
            {2925, 101},
            {3050, 106},
            {3187, 111},
            {3225, 116},
            {3350, 121}
        },
        0, // start speed
        0.0002 // tbh constant
    ));

    robot.intake.reset(new pros::MotorGroup({-11, 12}));
    robot.intake->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    robot.indexer.reset(new pros::ADIDigitalOut('A'));

    robot.roller.reset(new pros::Motor(5, pros::E_MOTOR_GEAR_100));
    robot.roller->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    robot.shooterFlap.reset(new pros::ADIDigitalOut('H'));

    robot.endgame.reset(new pros::ADIDigitalOut('B'));

    return robot;

}


Robot getRobot18(bool isSkills) {

    
    Robot robot;

    robot.drive.reset(new Drive(
        {-14, 11, -12, 13}, // left motor ports
        {-15, -2, 3, 4}, // right motor ports
        pros::E_MOTOR_GEAR_600, // internal gear ratio
        3.0/4.0, // external gear ratio
        2.74, // wheel diameter in inches
        14.25//15.2 // track width in inches
    ));

    robot.localizer.reset(new IMULocalizer(
        *robot.drive, // reference to drive object
        8, // imu port A
        9 // imu port B
    ));



    robot.flywheel.reset(new TBHFlywheel(
        {16, -19}, // ports
        { // rpm to volt data
            {1615, 5},
            {1966, 6},
            {2306, 7},
            {2646, 8},
            {3054, 9},
            {3416, 10},
            {3751, 11},
            {4141, 12}
        },
        { // rpm to distance data FOR FLAP DOWN
            {2450, 56},
            {2425, 61},
            {2475, 66},
            {2550, 71},
            {2575, 76},
            {2700, 81},
            {2800, 86},
            {2800, 91},
            {2850, 96},
            {2925, 101},
            {3050, 106},
            {3187, 111},
            {3225, 116},
            {3350, 121}
            
            
        },
        { // rpm to distance data FOR FLAP UP
            {2450, 56},
            {2425, 61},
            {2475, 66},
            {2550, 71},
            {2575, 76},
            {2700, 81},
            {2800, 86},
            {2800, 91},
            {2850, 96},
            {2925, 101},
            {3050, 106},
            {3187, 111},
            {3225, 116},
            {3350, 121}
        },
        0, // start speed
        0.0002 // tbh constant
    ));

    robot.intake.reset(new pros::MotorGroup({7, -5}));
    robot.intake->set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    robot.indexer.reset(new pros::ADIDigitalOut('H'));

    robot.roller.reset(new pros::Motor(10, pros::E_MOTOR_GEAR_100));
    robot.roller->set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);

    robot.shooterFlap.reset(new pros::ADIDigitalOut('G'));

    robot.endgame.reset(new pros::ADIDigitalOut('B'));

    return robot;

}