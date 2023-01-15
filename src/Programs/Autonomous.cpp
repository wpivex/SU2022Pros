#include "Subsystems/RobotBuilder.h"
#include "Programs/Driver.h"
#include "AutonomousFunctions/DriveFunctions.h"
#include "Algorithms/SingleBoundedPID.h"
#include "Algorithms/SimplePID.h"
#include "Algorithms/DoubleBoundedPID.h"
#include "Algorithms/NoPID.h"
#include "Algorithms/Alternator.h"
#include "Algorithms/Shooter.h"
#include "misc/MathUtility.h"
#include "misc/ProsUtility.h"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

#define GFU_DIST(maxSpeed) SingleBoundedPID({0.1, 0, 0, 0.12, maxSpeed})
#define GFU_DIST_PRECISE(maxSpeed) DoubleBoundedPID({0.1, 0, 0, 0.12, maxSpeed}, 0.1, 3)
#define GFU_TURN SimplePID({1, 0, 0.1, 0.0, 1})
#define GTU_TURN DoubleBoundedPID({1.25, 0, 0.095, 0.15, 1}, getRadians(1.5), 3)
#define GTU_TURN_PRECISE DoubleBoundedPID({1.25, 0, 0.095, 0.15, 1}, getRadians(0.75), 3)
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
    Shooter shooter;
    while (pros::millis() - start < 6000) {
        setEffort(*robot.intake, shooter.tickIntakeShootingSpeed(robot));
    }

    // reset indexer after 500ms, nonblocking
    pros::Task([&] {delayResetIndexer(robot); });
}

void threeTileAuton(Robot& robot) {
    setEffort(*robot.intake, 1);

    //todo

}

void twoTileAuton(Robot& robot) {// GENERATED C++ CODE FROM PathGen 3.4.3
// GENERATED C++ CODE FROM PathGen 3.4.3

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
// GENERATED C++ CODE FROM PathGen 3.4.3

// Robot assumes a starting position of (88.0,15.9) at heading of 42.48 degrees.
robot.flywheel->setVelocity(3148); // Preemptively set speed for next shot
setEffort(*robot.intake, 1); // Start running intake immediately
robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, 20.68, getRadians(42.48));
pros::delay(1000);
goTurnU(robot, GTU_TURN_PRECISE, getRadians(485.77));
goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, -18.01, getRadians(485.77));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(151.49));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 19.8, getRadians(151.49));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(72.16));

shoot(robot);
robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(133.26));
goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, 44.84, getRadians(133.26));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(48.36));

shoot(robot);
robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(356.35));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 10.12, getRadians(356.35));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(483.32));
goForwardU(robot, GFU_DIST_PRECISE(0.49), GFU_TURN, -18.77, getRadians(483.32));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(37.88));
goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, 4.9, getRadians(37.88));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(64.42));

shoot(robot);
robot.flywheel->setVelocity(3000); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(240.85));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 25.58, getRadians(240.85));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -24.92, getRadians(240.84));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(64.4));

shoot(robot);
robot.flywheel->setVelocity(3300); // Preemptively set speed for next shot

goTurnU(robot, GTU_TURN_PRECISE, getRadians(207.8));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 28.88, getRadians(207.8));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(241.69));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, -10.6, getRadians(241.69));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(184.18));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 8.83, getRadians(184.18));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(224.28));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, -12.0, getRadians(224.28));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(186.0));
goForwardU(robot, GFU_DIST_PRECISE(0.52), GFU_TURN, 8.21, getRadians(186.0));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(57.22));
goForwardU(robot, GFU_DIST_PRECISE(0.51), GFU_TURN, 15.08, getRadians(57.22));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(49.19));

shoot(robot);

// ================================================
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

void threeTile(Robot& robot){
    // GENERATED C++ CODE FROM PathGen 3.4.3

// Robot assumes a starting position of (16.3,90.8) at heading of 0.0 degrees.
// GENERATED C++ CODE FROM PathGen 3.4.3

// Robot assumes a starting position of (16.3,90.8) at heading of 0.0 degrees.
robot.flywheel->setVelocity(3290); // Preemptively set speed for next shot
setEffort(*robot.intake, 1); // Start running intake immediately
robot.drive->setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);


goTurnU(robot, GTU_TURN_PRECISE, getRadians(43.25));
setEffort(*robot.intake, 1);
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 19.9, getRadians(43.25));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -4.21, getRadians(403.24));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(324.2));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -19.46, getRadians(324.2));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(360.01));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, -5.18, getRadians(360.01));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 2.64, getRadians(359.99));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(312.45));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 20.86, getRadians(312.45));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(15.00));
pros::delay(500);

shoot(robot);
robot.flywheel->setVelocity(3275); // Preemptively set speed for next shot

setEffort(*robot.intake, 1);
goTurnU(robot, GTU_TURN_PRECISE, getRadians(312.08));
goForwardU(robot, GFU_DIST_PRECISE(1), GFU_TURN, 11.63, getRadians(312.08));
goTurnU(robot, GTU_TURN_PRECISE, getRadians(21.88));
pros::delay(500);

shoot(robot);

// ================================================


}