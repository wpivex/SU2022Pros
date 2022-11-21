#include "main.h"

#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"

Robot robot = getRobot();
Driver driver(robot);

#define RUN_AUTON
using namespace pros;

void initialize() {
	if (robot.localizer) robot.localizer->init();
}

void disabled() {}


void competition_initialize() {}


void autonomous() {

}


void opcontrol() {

	#ifdef RUN_AUTON
	autonomous();
	return;
	#endif

	driver.runDriver();
}
