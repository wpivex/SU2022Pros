#include "main.h"

#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"

Robot robot = getRobot();
Driver driver(robot, TANK_DRIVE);

#define RUN_AUTON

using namespace pros;

void initialize() {
	if (robot.localizer) robot.localizer->init();
}

void disabled() {}


void competition_initialize() {}


void autonomous() {
	#define GFU_DISTANCE SingleBoundedPID({0.1, 0, 0, 0.1, 1})
	#define GFU_TURN SimplePID({1, 0, 0, 0.1, 1})
	#define GTU_Turn DoubleBoundedPID({1, 0, 0}, getRadians(2), 1)

	robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

	pros::lcd::initialize();
	goForwardU(robot, GFU_DISTANCE, GFU_TURN, 24);

	pros::lcd::print(0, "done");

}


void opcontrol() {

	
    if (false && robot.flywheel) {
        pros::Task taskFlywheel([&] {
            robot.flywheel->maintainVelocityTask();
        });
    }

    if (false && robot.localizer) {
        pros::Task taskLocalizer([&] {
            robot.localizer->updatePositionTask();
        });
    }


	#ifdef RUN_AUTON
	autonomous();
	return;
	#endif

	driver.runDriver();
}
