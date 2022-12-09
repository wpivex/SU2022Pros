#include "main.h"

#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "misc/SDCardReader.h"

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

	// #ifdef RUN_AUTON
	// autonomous();
	// return;
	// #endif

	// driver.runDriver();

	SDCardReader sdcr = SDCardReader();
	std::vector<std::vector<Position*>> output = sdcr.readSDCardData("test.csv");

    // for (int i = 0; i < 1; i++) {
    //     for (int j = 0; j < output[i].size(); j++) {
    //         pros::screen::print(TEXT_MEDIUM, i, "X: %f.3, Y: %f.3", (*output[i][j]).x, (*output[i][j]).y);
    //     }
    // }
}
