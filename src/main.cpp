#include "main.h"


#define RUN_AUTON
using namespace pros;

void initialize() {
	printf("init\n");
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

	printf("teleop\n");
}
