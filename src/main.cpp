#include "main.h"

#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "Programs/Autonomous.h"


Robot robot = getRobot();
Driver driver(robot, TANK_DRIVE);

#define RUN_AUTON

using namespace pros;

void initialize() {
	pros::lcd::initialize();
	if (robot.localizer) robot.localizer->init();
	pros::lcd::print(0, "initialized");
}

void disabled() {}


void competition_initialize() {}


void autonomous() {

	matchAutonIMUOnly(robot);
	
}


void opcontrol() {

	
    if (true && robot.flywheel) {
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
