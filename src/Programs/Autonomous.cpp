#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"
#include "Algorithms/NoPID.h"
#include "Algorithms/Alternator.h"
#include "misc/MathUtility.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.12, maxSpeed})
#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.1, 0, 0, 0.12, maxSpeed}, 0.1, 3)
#define GFU_TURN SimplePID({1, 0, 0.1, 0.0, 1})
#define GTU_TURN DoubleBoundedPID({1.25, 0, 0.09, 0.14, 1}, getRadians(1.5), 3)
#define GTU_TURN_PRECISE DoubleBoundedPID({1.25, 0, 0.09, 0.14, 1}, getRadians(0.75), 3)
#define GCU_CURVE SimplePID({2.5/*2.25*//*1.7*/, 0, 0})

// don't stop motors at end
#define NO_SLOWDOWN(maxSpeed) NoPID(maxSpeed)

void startIntake(Robot& robot) {
    pros::delay(300);
    setEffort(*robot.intake, 1);
}

void delayResetIndexer(Robot& robot) {
    pros::delay(500);
    robot.indexer->set_value(false);
    setEffort(*robot.intake, 1);
}

// shoot a 3-burst round. First two rounds are short burst (110ms with 220ms break), third is longer (300ms)
void shoot(Robot& robot) {

    // cock gun
    setEffort(*robot.intake, 1);
    robot.indexer->set_value(true);
    pros::delay(500);

    // wait for spinup
    if (!robot.flywheel->atTargetVelocity()) {

        constexpr uint32_t TIMEOUT_MS = 5000; // maximum time to wait for spinup to proper velocity

        setEffort(*robot.intake, 0);
        uint32_t start = pros::millis();
        while (!robot.flywheel->atTargetVelocity() && pros::millis() - start < TIMEOUT_MS) {
            pros::delay(10);
        }
    }

    // first shot
    setEffort(*robot.intake, -1);
    pros::delay(110);
    robot.intake->brake();
    robot.flywheel->setVelocity(3300);
    pros::delay(280);

    // second shot
    setEffort(*robot.intake, -1);
    pros::delay(110);
    robot.intake->brake();
    pros::delay(280);

    // third shot (longer)
    setEffort(*robot.intake, -1);
    pros::delay(300);
    robot.intake->brake();

    // reset indexer after 500ms, nonblocking
    pros::Task([&] {delayResetIndexer(robot); });
}

void simple18Auton(Robot& robot) {
    // GENERATED C++ CODE FROM PathGen 3.4.1

    // Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
    // GENERATED C++ CODE FROM PathGen 3.4.1

    // Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
    //robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot
    //robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    //robot.intake->move_voltage(12000);
    //robot.localizer->setPosition(88, 15.9);

// ================================================

// GENERATED C++ CODE FROM PathGen 3.4.1

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
// GENERATED C++ CODE FROM PathGen 3.4.1

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
//robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot
//robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
//Saved as saves/save7_v3_4_1.pg3!
// GENERATED C++ CODE FROM PathGen 3.4.1

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
// GENERATED C++ CODE FROM PathGen 3.4.1

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
//robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot
//robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

//Saved as saves/save7_v3_4_1.pg3!
// GENERATED C++ CODE FROM PathGen 3.4.1

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
// GENERATED C++ CODE FROM PathGen 3.4.1

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
//robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot
robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 20.68, getRadians(42.48));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(476.85));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -19.5, getRadians(476.85));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(141.57));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 25.23, getRadians(141.57));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(69.06));

shoot(robot);
//robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(135.66));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 39.73, getRadians(135.66));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(48.15));

shoot(robot);
//robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(6.38));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 13.03, getRadians(6.38));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(474.62));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -17.02, getRadians(474.62));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(30.96));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 5.01, getRadians(30.96));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(63.41));

shoot(robot);
//robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(206.73));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 32.0, getRadians(206.73));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(224.34));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -13.22, getRadians(224.34));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(189.65));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 10.24, getRadians(189.65));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(223.73));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -13.68, getRadians(223.73));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(183.81));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 9.69, getRadians(183.81));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(52.2));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 15.77, getRadians(52.2));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(49.19));

shoot(robot);
}

void matchAutonIMUOnly(Robot& robot) {

	robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    simple18Auton(robot);

	//robot.flywheel->setVelocity(3250);
    //robot.localizer->setHeading(0);
    //goCurveU(robot, GFU_DIST_PRECISE(0.4), GCU_CURVE, getRadians(0), getRadians(180), 12);

    // GENERATED C++ CODE FROM PathGen 3.0

    // Robot assumes a starting position of (71.7,48.5) at heading of 90.0 degrees.
    //robot.localizer->setHeading(getRadians(90.0));
/*
    goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, 24.06, getRadians(90.0));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(135.0));
    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 34.23, getRadians(135.0));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(180.17));
    goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, 23.92, getRadians(180.17));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(269.66));
    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 24.06, getRadians(269.66));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(0.17));
    goForwardU(robot, NO_SLOWDOWN(0.4), GFU_TURN, 24.13, getRadians(0.17));
    goCurveU(robot, NO_SLOWDOWN(0.4), GCU_CURVE, getRadians(0.17), getRadians(89.32), 23.99);
    goForwardTimedU(robot, GFU_TURN, 1, 0.4, getRadians(89.32));
    // ================================================
*/
}

void skillsAutonIMUOnly(Robot& robot) {

}

void twoTileAuton(Robot& robot){
    goForwardU(robot, GFU_DIST_PRECISE(0.5), GFU_TURN, 24, getRadians(0));
}

void redOnBlueSkillsAuton(Robot& robot){
        
    // Robot assumes a starting position of (16.5,66.4) at heading of 359.66 degrees.
    // GENERATED C++ CODE FROM PathGen 3.4.2

    // Robot assumes a starting position of (16.5,66.4) at heading of 359.66 degrees.
    robot.flywheel->setVelocity(2800); // Preemptively set speed for next shot
    robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);       

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(270.0));        
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 8.59, getRadians(270.0));

    shoot(robot);
    robot.flywheel->setVelocity(2800); // Preemptively set speed for next shot

    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -26.83, getRadians(269.99));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(360.0));        
    setEffort(*robot.intake, 1);
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 18.55, getRadians(360.0));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(270.0));        
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 19.98, getRadians(270.0));
    setEffort(*robot.intake, 0);
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(-108.83));      

    shoot(robot);
    robot.flywheel->setVelocity(2800); // Preemptively set speed for next shot

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(350.46));       
    setEffort(*robot.intake, 1);
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 24.62, getRadians(350.46));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(315.25));       
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 34.18, getRadians(315.25));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(221.6));        
    setEffort(*robot.intake, 0);
    goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 35.92, getRadians(221.6));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(181.43));       

    shoot(robot);

    // ================================================

}

void topSideSkillsAuto(Robot& robot){
        
    // GENERATED C++ CODE FROM PathGen 3.4.2

    // Robot assumes a starting position of (16.6,104.8) at heading of 0.0 degrees.
    // GENERATED C++ CODE FROM PathGen 3.4.2

    // Robot assumes a starting position of (16.6,104.8) at heading of 0.0 degrees.
    robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot
    robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);       

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(66.76));        
    setEffort(*robot.intake, 1);
    pros::delay(260);
    goForwardU(robot, GFU_DIST_PRECISE(0.4), GFU_TURN, 17.15, getRadians(66.76));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(359.54));       
    setEffort(*robot.intake, 0);
    goForwardU(robot, GFU_DIST_PRECISE(0.4), GFU_TURN, -16.7, getRadians(359.54));


    // [run roller for 180 distance at 1 speed]


    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 19.43, getRadians(359.53));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(270.07));       
    goForwardU(robot, GFU_DIST_PRECISE(0.4), GFU_TURN, -17.35, getRadians(270.07));


    // [run roller for 180 distance at 1 speed]


    goForwardU(robot, GFU_DIST_PRECISE(0.78), GFU_TURN, 9.4, getRadians(270.06));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(0.56));
    goForwardU(robot, GFU_DIST_PRECISE(0.4), GFU_TURN, 13.96, getRadians(0.56));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(2.09));

    shoot(robot);
    robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

    setEffort(*robot.intake, 1);
    pros::delay(260);
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(314.45));       
    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 79.0, getRadians(314.45));
    setEffort(*robot.intake, 0);
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(58.51));        

    shoot(robot);
    robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

    setEffort(*robot.intake, 1);
    pros::delay(260);
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(314.44));       
    goForwardU(robot, GFU_DIST_PRECISE(0.4), GFU_TURN, 17.66, getRadians(314.44));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(71.48));        
    setEffort(*robot.intake, 0);

    shoot(robot);
    robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(177.79));       
    setEffort(*robot.intake, 1);
    pros::delay(260);
    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 24.12, getRadians(177.79));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(314.87));       
    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 68.21, getRadians(314.87));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(89.68));        
    setEffort(*robot.intake, 0);
    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 75.63, getRadians(89.68));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(90.28));        

    shoot(robot);
    robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

    setEffort(*robot.intake, 1);
    pros::delay(260);
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(180.01));       
    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 76.91, getRadians(180.01));
    setEffort(*robot.intake, 0);
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(-119.29));      

    shoot(robot);
    robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

    goTurnU(robot, GTU_TURN_PRECISE, getRadians(270.0));        
    setEffort(*robot.intake, 1);
    pros::delay(260);
    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 76.91, getRadians(270.0));
    goTurnU(robot, GTU_TURN_PRECISE, getRadians(173.81));       
    setEffort(*robot.intake, 0);

    shoot(robot);

    pros::delay(260);
    pros::lcd::print(2, "heading =  ", robot.localizer->getHeading());

    // ================================================


}