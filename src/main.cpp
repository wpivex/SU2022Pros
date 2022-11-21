#include "main.h"
#include "misc/Logger.h"
#include "misc/Grapher.h"
#include <string>

#include "pros/screen.hpp"

#define RUN_AUTON
using namespace pros;

void initialize() {
	printf("init\n");
}

void disabled() {}


void competition_initialize() {}

void test() {

	pros::screen::set_pen(COLOR_WHITE);
        

        std::string mins = std::to_string(1);
        std::string maxs = std::to_string(2);
        pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL, 5, GRAPH_BOTTOM, mins.c_str());
        pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL, 5, GRAPH_TOP, maxs.c_str());

	while (true) {
		pros::delay(10);
	}
}
void autonomous() {
		
	pros::screen::erase();
	pros::screen::set_pen(COLOR_WHITE);
	pros::screen::print(pros::text_format_e_t::E_TEXT_SMALL, 1, "fuck%f",0.123);


	pros::delay(2000);
    graphy::AsyncGrapher grapher(2,10);

    grapher.addDataType("Ema Vel", COLOR_ORANGE);
    grapher.addDataType("Desired Vel", COLOR_AQUAMARINE);
    grapher.addDataType("Kalman Vel", COLOR_RED);
    grapher.startTask();
    pros::ADIAnalogIn pot('A');
    while(true) {
        grapher.update("Desired Vel", 3);
        grapher.update("Kalman Vel", 5);
        grapher.update("Ema Vel", 7);
        pros::delay(10);
    }


}


void opcontrol() {

	#ifdef RUN_AUTON
	autonomous();
	return;
	#endif

	printf("teleop\n");
}
