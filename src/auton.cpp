#include "auton.h"

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({16, 17, 18}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 11, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::ControllerSettings lateralPID(8, 0, 10, 3, 1, 100, 3, 500, 40);
lemlib::ControllerSettings angularPID(6, 0, 45, 3, 1, 100, 3, 500, 0); 

pros::Imu imu(4);
pros::Rotation horizontalRotation(14);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_2, 0.25);
lemlib::OdomSensors odometry(nullptr, nullptr, &horizontalWheel, nullptr, &imu);

lemlib::ExpoDriveCurve lateralCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve angularCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry, &lateralCurve, &angularCurve);

pros::adi::Pneumatics lift('A', false);
pros::adi::Pneumatics bump('B', false);
pros::adi::Pneumatics tongue('C', false);

pros::Distance left(21);
pros::Distance right(1);

int state = 2;

void drive(int speed, int ms) {
    chassis.tank(speed, speed, true);
    pros::delay(ms);
    chassis.tank(0, 0, true);
}

void longGoal(int ms) {
    state = 1;
    pros::delay(ms);
    intake.brake();
    state = 2;
}

void midGoal(int ms) {
    state = 0;
    pros::delay(ms);
    intake.brake();
    state = 2;
}

void lowGoal(int ms) {
    intake.move(-80);
    pros::delay(ms);
    intake.brake();
}

void rightWing() {
    chassis.setPose(55 - inch(left.get()), 20, chassis.getPose().theta);
    chassis.moveToPoint(38, 5, 500, {}, false);
    tongue.retract();
    chassis.moveToPoint(49.5, 25, 750, {.forwards=false, .maxSpeed=80}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.turnToHeading(180, 250, {}, false);
    chassis.moveToPoint(49.5, 40, 1000000, {.forwards=false});
    chassis.waitUntilDone();
}
 
void leftWing() {
    chassis.setPose(-56 + inch(right.get()), 20, chassis.getPose().theta);
    chassis.moveToPoint(-38, 5, 500, {.maxSpeed=80}, false);
    tongue.retract();
    chassis.moveToPoint(-26, 25, 750, {.forwards=false, .maxSpeed=80}, false);
    chassis.turnToHeading(-180, 250, {}, false);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.moveToPoint(-26, 40, 1000000, {.forwards=false, .maxSpeed=80});
    chassis.waitUntilDone();
}

void right4() {
    in();
    chassis.moveToPoint(12, 28, 1250, {.maxSpeed=80});
    pros::delay(700);
    chassis.waitUntilDone();
    tongue.retract();

    chassis.turnToHeading(142, 750, {}, false);
    chassis.moveToPoint(36.5, 10, 1000, {.maxSpeed=100}, false);
    lift.extend();
    chassis.turnToHeading(180, 500, {.maxSpeed=80}, false);
    hood.extend();
    chassis.setPose(50 - inch(left.get()), chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPoint(31, 28, 750, {.forwards=false, .maxSpeed=80}, false);
    chassis.tank(-50, -50);
    longGoal(500);
}

void left4() {
    in();
    chassis.moveToPoint(-11.5, 28, 1250, {.maxSpeed=80});
    pros::delay(700);
    chassis.waitUntilDone();
    tongue.retract();

    chassis.turnToHeading(-142, 750, {}, false);
    chassis.moveToPoint(-36.5, 6, 1000, {.maxSpeed=100}, false);
    lift.extend();
    chassis.turnToHeading(-180, 500, {.maxSpeed=80}, false);
    hood.extend();
    chassis.setPose(-50 + inch(right.get()), chassis.getPose().y, chassis.getPose().theta);
    chassis.moveToPoint(-31, 28, 750, {.forwards=false, .maxSpeed=80}, false);
    chassis.tank(-50, -50);
    longGoal(500);
}

void right6() {
    in();
    chassis.moveToPoint(12, 28, 1000, {.maxSpeed=80});
    pros::delay(750);
    chassis.waitUntilDone();

    chassis.turnToHeading(60, 500, {.maxSpeed=80}, false);
    tongue.retract();
    chassis.moveToPoint(33, 41.5, 750, {.maxSpeed=100});
    pros::delay(500);
    tongue.extend();
    chassis.waitUntilDone();
    lift.extend();

    chassis.moveToPoint(30, 35, 750, {.forwards=false, .minSpeed=80, .earlyExitRange=2});
    chassis.moveToPoint(35, 16, 750, {.forwards=false});
    tongue.retract();
    chassis.waitUntilDone();
    chassis.swingToHeading(180, DriveSide::RIGHT, 750);
    chassis.waitUntilDone();
    chassis.tank(-50, -50);
    hood.extend();
    longGoal(600);

    chassis.turnToHeading(180, 500);
    rightWing();
}

void left6() {
    in();
    chassis.moveToPoint(-12, 28, 1000, {.maxSpeed=80});
    pros::delay(750);
    chassis.waitUntilDone();

    chassis.turnToHeading(-60, 500, {.maxSpeed=80}, false);
    tongue.retract();
    chassis.moveToPoint(-33, 41, 750, {.maxSpeed=100});
    pros::delay(500);
    tongue.extend();
    chassis.waitUntilDone();
    lift.extend();

    chassis.moveToPoint(-32, 35, 750, {.forwards=false, .minSpeed=80, .earlyExitRange=2});
    chassis.moveToPoint(-39, 18, 750, {.forwards=false});
    tongue.retract();
    chassis.waitUntilDone();
    chassis.swingToHeading(-180, DriveSide::LEFT, 750);
    chassis.waitUntilDone();
    chassis.tank(-50, -50);
    hood.extend();
    longGoal(600);
    
    chassis.turnToHeading(-180, 500);
    leftWing();
}

void rightSplit() {
    right4();
    tongue.extend();

    chassis.turnToHeading(180, 500, {}, false);
    chassis.setPose(50 - inch(left.get()), 30, chassis.getPose().theta);
    in();
    chassis.moveToPoint(30.25, -10.5, 2000, {.maxSpeed=50}, false);

    chassis.moveToPoint(30.25, 6.5, 750, {.forwards=false, .maxSpeed=100}, false);
    chassis.turnToHeading(180, 500, {}, false);
    chassis.setPose(50 - inch(left.get()), chassis.getPose().y, chassis.getPose().theta);
    chassis.turnToHeading(-45, 750, {.maxSpeed=80}, false);
    tongue.retract();
    chassis.moveToPoint(-4, 42.5, 1500, {.maxSpeed=100}, false);
    lowGoal(1000);

    if (variation == 1) { // Wing
        chassis.moveToPoint(18, 15, 1000, {.forwards=false, .maxSpeed=100}, false);
        chassis.turnToHeading(0, 500, {.maxSpeed=80}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(18.5, 44, 1500, {}, false);
        chassis.turnToHeading(-25, 1000, {}, false);
    } else if (variation == 2) { // Stay
        drive(-50, 200);
        pros::delay(500);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        drive(127, 200);
    }
}

void leftSplit() {
    left4();
    tongue.extend();

    chassis.turnToHeading(180, 500, {}, false);
    chassis.setPose(-50 + inch(right.get()), 30, chassis.getPose().theta);
    in();
    chassis.moveToPoint(-32.5, -10.5, 2000, {.maxSpeed=50}, false);

    chassis.moveToPoint(-32.5, 14, 500, {.forwards=false, .maxSpeed=100}, false);
    chassis.turnToHeading(180, 500, {}, false);
    chassis.setPose(-50 + inch(right.get()), chassis.getPose().y, chassis.getPose().theta);
    chassis.turnToHeading(-145, 750, {.maxSpeed=100}, false);
    lift.retract();
    bump.extend();
    tongue.retract();
    chassis.moveToPoint(5.5, 45, 1500, {.forwards=false, .maxSpeed=100}, false);
    midGoal(1000);

    if (variation == 1) { // Wing
        chassis.moveToPoint(-17.5, 24, 1000, {.maxSpeed=100}, false);
        lift.extend();
        chassis.turnToHeading(180, 500, {.maxSpeed=80}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(-17.5, 46, 1000000, {.forwards=false});
        chassis.waitUntilDone();
    } else if (variation == 2) { // Stay
        bump.retract();
        drive(50, 200);
        pros::delay(500);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        drive(-127, 200);
    }
}

void awpWing() {
    if (variation == 1) { // Right Goal
        pros::delay(5000);
        chassis.moveToPoint(0, -80, 2500, {.forwards=false, .maxSpeed=80}, false);
        lift.extend();
        chassis.turnToHeading(-90, 1000, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed=80}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(32, -81, 1000000, {.forwards=false}, false);
    } else if (variation == 2) { // Left Goal
        pros::delay(10000);
        chassis.moveToPoint(0, 20, 1000, {.maxSpeed=80}, false);
        lift.extend();
        chassis.turnToHeading(-90, 1000, {.maxSpeed=80}, false);
        chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
        chassis.moveToPoint(32, 19, 1000000, {.forwards=false}, false);
    }
}

void autonSkills() {
    drive(-50, 150);
    tongue.extend();
    drive(127, 450);
}
