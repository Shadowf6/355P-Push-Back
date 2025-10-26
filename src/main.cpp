#include "main.h"

pros::MotorGroup leftMotors({-1, 13, -12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({8, -9, 10}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 15, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(3);
pros::Rotation verticalRotation(-7);
pros::Rotation horizontalRotation(20);
lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_275, 1);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_275, 3);
lemlib::OdomSensors odometry(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);

lemlib::ControllerSettings lateralPID(5, 0, 10, 3, 1, 100, 3, 500, 6.7);
lemlib::ControllerSettings angularPID(4, 0, 11, 3, 1, 100, 3, 500, 0); 

lemlib::ExpoDriveCurve lateralCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve angularCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry, &lateralCurve, &angularCurve);

pros::Motor intake(15, pros::MotorGearset::blue);
pros::Motor score(-2, pros::MotorGearset::blue);

pros::adi::Pneumatics pivot('A', false);
pros::adi::Pneumatics tongue('B', false);
pros::adi::Pneumatics elbow('C', false);

lv_obj_t *screen;


void initialize() {
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    lv_init();
    screen = lv_screen_active();
    createDisplay(screen);

    pros::Task display([&]() {
        while (true) {
            lv_timer_handler();

            auto pose = chassis.getPose();
            updateCoords(pose.x, pose.y, pose.theta);

            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {
    horizontalRotation.reset_position();
    horizontalWheel.reset();
    verticalRotation.reset_position();
    verticalWheel.reset();
    imu.tare_heading();
}

void autonomous() {
    chassis.setPose(0, 0, 0);

    auto in([&]() {intake.move(127); score.move(-40);});
    auto reset([&]() {intake.move(0); score.move(0);});
    auto goal([&]() {score.move(127);});
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed, true); pros::delay(ms); chassis.tank(0, 0, true);});

    if (auton == 1) {
        // Left (AWP)
        in();
        chassis.moveToPoint(-12, 26, 2000, {.maxSpeed=50});
        pros::delay(1100);
        tongue.extend();
        pros::delay(500);
        chassis.waitUntilDone();

        chassis.moveToPoint(-8.5, 22.5, 500, {.maxSpeed=60}, false);
        chassis.turnToHeading(-135, 750, {.maxSpeed=60}, false);
        chassis.moveToPoint(5, 40, 1200, {.forwards=false, .maxSpeed=60}, false);
        chassis.turnToHeading(-135, 250);
        pivot.extend();
        goal();
        pros::delay(450);
        pivot.retract();
        reset();

        chassis.moveToPoint(-34.85, 2, 2000, {.maxSpeed=60}, false);
        in();
        chassis.turnToHeading(-179, 750, {.maxSpeed=60}, false);
        drive(50, 1200);
        drive(-80, 150);
        pros::delay(500);
        drive(80, 750);
        pros::delay(150);
        drive(-80, 150);
        pros::delay(500);

        chassis.moveToPose(-30, 90, -175, 2000, {.forwards=false, .maxSpeed=50}, false);
        goal();
    } else if (auton == 2) { 
        // Right (Elim)
        in();
        chassis.moveToPoint(11, 26, 2500, {.maxSpeed=50});
        pros::delay(1100);
        tongue.extend();
        pros::delay(500);
        chassis.waitUntilDone();

        chassis.turnToHeading(142, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(35.5, -0.25, 2000, {.maxSpeed=60}, false);
        chassis.turnToHeading(173, 1000, {.maxSpeed=50}, false);
        drive(50, 1400);
        drive(-80, 150);
        pros::delay(500);
        drive(80, 750);
        pros::delay(150);
        drive(-80, 150);
        pros::delay(500);

        chassis.moveToPose(37, 90, -175, 2500, {.forwards=false, .maxSpeed=50}, false);
        goal();
    } else if (auton == 3) {
        // Skills
        in();
        chassis.moveToPoint(-2.5, 2.6, 500, {.earlyExitRange=2});
        chassis.moveToPoint(-9, 12.5, 500, {.minSpeed=20, .earlyExitRange=2});
        chassis.moveToPose(-11, 26, 0, 500, {.minSpeed=40, .earlyExitRange=3});
        chassis.moveToPose(-13, 26, 0, 1000, {.minSpeed=60, .earlyExitRange=4});
        chassis.waitUntilDone();
        tongue.extend();
        pros::delay(2000);

        chassis.moveToPoint(-8.5, 22.5, 500, {.forwards=false, .maxSpeed=50}, false);
        chassis.turnToHeading(-135, 750, {.maxSpeed=70}, false);
        chassis.moveToPoint(5, 38, 1200, {.forwards=false, .maxSpeed=50}, false);
        chassis.turnToHeading(-130, 250);
        pivot.extend();
        score.move(100);
        pros::delay(2500);
        pivot.retract();
        reset();

        chassis.moveToPoint(-34, -2, 2000, {.maxSpeed=60}, false);
        in();
        chassis.turnToHeading(-179, 1000, {.maxSpeed=50}, false);
        drive(50, 1800);
        drive(-80, 150);
        pros::delay(500);
        drive(70, 1600);
        pros::delay(250);
        drive(-70, 150);
        pros::delay(500);

        chassis.moveToPose(-33, 90, -179, 2500, {.forwards=false, .maxSpeed=50}, false);
        goal();
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
            // Intake
            pivot.retract();
            intake.move(127);   
            score.move(-40);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { 
            // Outtake
            intake.move(-127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { 
            // Long Goal
            pivot.retract();
            intake.move(127);
            score.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // Middle Goal
            pivot.extend();
            intake.move(127);
            score.move(127);
        } else {
            intake.move(0);
            score.move(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            // Elbow
            if (elbow.is_extended()) elbow.retract();
            else elbow.extend();
            pros::delay(250);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            // Tongue
            if (tongue.is_extended()) tongue.retract();
            else tongue.extend();
            pros::delay(250);
        }

        pros::delay(20);
    }
}
