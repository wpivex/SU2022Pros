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

    uint32_t start = pros::millis();
    while (pros::millis() - start < 3000) {
        if (robot.flywheel->getTargetVelocity() - robot.flywheel->getCurrentVelocity() > 75) {
            setEffort(*robot.intake, 0);
        } else {
            setEffort(*robot.intake, -0.5);
        }
    }

    // reset indexer after 500ms, nonblocking
    pros::Task([&] {delayResetIndexer(robot); });
}

void twoTileAuton(Robot& robot) {

setEffort(*robot.intake, 1);
robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot
robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, 20.68, getRadians(42.48));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(485.77));
goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, -18.01, getRadians(485.77));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(149.04));
goForwardU(robot, GFU_DIST_PRECISE(0.8), GFU_TURN, 25.06, getRadians(149.04));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(69.06));

shoot(robot);
robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot
pros::lcd::print(5, "final heading: %f", robot.localizer->getHeading());
/*
goTurnU(robot, GTU_TURN_PRECISE, getRadians(135.66));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 39.73, getRadians(135.66));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(48.15));

shoot(robot);
robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(6.38));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 13.03, getRadians(6.38));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(474.62));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -17.02, getRadians(474.62));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(30.96));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 5.01, getRadians(30.96));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(63.41));

shoot(robot);
robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(210.95));
goForwardU(robot, GFU_DIST_PRECISE(0.75), GFU_TURN, 28.16, getRadians(210.95));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(241.69));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -10.6, getRadians(241.69));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(184.18));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 8.83, getRadians(184.18));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(224.28));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -12.0, getRadians(224.28));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(186.0));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 8.21, getRadians(186.0));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(57.22));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 15.08, getRadians(57.22));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(49.19));

shoot(robot);
*/}


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
    goForwardU(robot, GFU_DIST_PRECISE(0.4), GFU_TURN, -17.7, getRadians(359.54));


    // [run roller for 180 distance at 1 speed]


    goForwardU(robot, GFU_DIST_PRECISE(0.6), GFU_TURN, 20.43, getRadians(359.53));
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

    // ================================================


}