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

lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry);

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
    auto goal([&]() {intake.move(127); score.move(127);});
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed, true); pros::delay(ms); chassis.tank(0, 0, true);});

    if (auton == 1) { // Left (AWP)
        /*
        in();
        chassis.moveToPoint(-12, 26, 2000, {.maxSpeed=50});
        pros::delay(1200);
        tongue.extend();
        pros::delay(500);
        chassis.waitUntilDone();

        chassis.moveToPoint(-8.5, 22.5, 500, {.maxSpeed=60}, false);
        chassis.turnToHeading(-135, 750, {.maxSpeed=60}, false);
        chassis.moveToPoint(3, 38, 1000, {.forwards=false, .maxSpeed=60}, false);
        chassis.turnToHeading(-132, 250);
        pivot.extend();
        goal();
        pros::delay(300);
        pivot.retract();
        reset();
        pros::delay(500);

        chassis.moveToPoint(-35, 2, 2000, {.maxSpeed=60}, false);
        in();
        chassis.turnToHeading(-179, 750, {.maxSpeed=60}, false);
        drive(50, 1300);
        drive(-80, 150);
        pros::delay(500);

        reset();
        tongue.retract();
        chassis.moveToPose(-29.5, 90, -175, 1700, {.forwards=false, .maxSpeed=50}, false);
        goal();*/

        in();
        chassis.moveToPoint(-12, 26, 2000, {.maxSpeed=50});
        pros::delay(1200);
        tongue.extend();
        pros::delay(500);
        chassis.waitUntilDone();

        chassis.moveToPoint(-8.5, 22.5, 500, {.maxSpeed=60}, false);
        chassis.turnToHeading(-135, 750, {.maxSpeed=60}, false);
        chassis.moveToPoint(3, 38, 1000, {.forwards=false, .maxSpeed=60}, false);
        chassis.turnToHeading(-130, 250);
        pivot.extend();
        goal();
        pros::delay(300);
        pivot.retract();
        reset();
        pros::delay(500);

        chassis.moveToPoint(-37, 2, 2500, {.maxSpeed=60}, false);
        in();
        chassis.turnToHeading(-179, 750, {.maxSpeed=60}, false);
        drive(50, 1300);
        drive(-80, 150);
        pros::delay(500);

        reset();
        tongue.retract();
        chassis.moveToPose(-29.5, 90, -175, 1700, {.forwards=false, .maxSpeed=50}, false);
        goal();
    } else if (auton == 2) { // Right (Elim)
        in();
        chassis.moveToPoint(12, 26, 2500, {.maxSpeed=50});
        pros::delay(1200);
        tongue.extend();
        pros::delay(500);
        chassis.waitUntilDone();

        chassis.turnToHeading(142, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(35.75, -0.25, 2000, {.maxSpeed=60}, false);
        chassis.turnToHeading(-180, 1000, {.maxSpeed=50}, false);
        drive(50, 1400);
        drive(-80, 150);
        pros::delay(500);

        reset();
        chassis.moveToPose(33.67, 90, 175, 1700, {.forwards=false, .maxSpeed=50}, false);
        goal();
        pros::delay(1400);
        reset();
        pros::delay(100);
        goal();
        pros::delay(1600);
        intake.brake();
        drive(80, 150);
        pros::delay(500);
        drive(-127, 1000);
    } else if (auton == 3) { // Left (Elim)
        in();
        chassis.moveToPoint(-12, 26, 2000, {.maxSpeed=50});
        pros::delay(1200);
        tongue.extend();
        pros::delay(500);
        chassis.waitUntilDone();

        chassis.turnToHeading(-135, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(-35.5, 2, 2500, {.maxSpeed=60}, false);
        in();
        chassis.turnToHeading(179, 750, {.maxSpeed=60}, false);
        drive(50, 1400);
        drive(-80, 150);
        pros::delay(500);

        reset();
        tongue.retract();
        chassis.moveToPose(-31, 90, -175, 1500, {.forwards=false, .maxSpeed=50}, false);
        goal();
        pros::delay(1300);
        reset();
        pros::delay(100);
        goal();
        pros::delay(1900);
        intake.brake();
        drive(80, 150);
        pros::delay(500);
        drive(-127, 1000);
    } else if (auton == 4) { // Left (Scuffed)
        in();
        chassis.moveToPoint(-12, 26, 2000, {.maxSpeed=50});
        pros::delay(1200);
        tongue.extend();
        pros::delay(500);
        chassis.waitUntilDone();

        chassis.turnToHeading(-135, 1000, {.maxSpeed=60}, false);
        chassis.moveToPoint(-38.5, 2, 2500, {.maxSpeed=60}, false);
        in();
        chassis.turnToHeading(179, 750, {.maxSpeed=60}, false);
        drive(50, 1400);
        drive(-80, 150);
        pros::delay(500);

        reset();
        tongue.retract();
        chassis.moveToPose(-31, 90, -175, 1500, {.forwards=false, .maxSpeed=50}, false);
        goal();
        pros::delay(1300);
        reset();
        pros::delay(100);
        goal();
        pros::delay(1900);
        intake.brake();
        drive(80, 150);
        pros::delay(500);
        drive(-127, 1000);
    } else if (auton == 5) { // Skills (Park)
        drive(50, 350);
        pros::delay(500);
        drive(-127, 700);
    } else if (auton == 6) { // Skills (Actual Route)
        in();
        chassis.moveToPoint(-7, 7, 500, {.earlyExitRange=2});
        chassis.moveToPoint(-12, 24, 1000, {.minSpeed=55});
        chassis.waitUntilDone();
        pros::delay(500);

        chassis.moveToPoint(-8.5, 22.5, 500, {.forwards=false, .maxSpeed=50}, false);
        chassis.turnToHeading(-135, 750, {.maxSpeed=70}, false);
        chassis.moveToPoint(5, 39, 1200, {.forwards=false, .maxSpeed=50}, false);
        chassis.turnToHeading(-135, 250);
        pivot.extend();
        score.move(100);
        pros::delay(2500);
        pivot.retract();
        reset();

        chassis.moveToPoint(-33.5, -2, 2000, {.maxSpeed=60}, false);
        tongue.extend();
        chassis.turnToHeading(-179, 1000, {.maxSpeed=50}, false);
        in();
        drive(50, 2000);
        pros::delay(500);
        drive(-80, 150);
        pros::delay(500);

        chassis.moveToPose(-33.25, 90, -179, 1500, {.forwards=false, .maxSpeed=50}, false);
        score.move(100);
        pros::delay(3500);
        reset();
        drive(80, 150);
        pros::delay(500);
        drive(-127, 1000);
        tongue.retract();

        chassis.moveToPoint(-22, -9, 1000, {}, false);
        chassis.turnToHeading(-76, 2000, {.direction=AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed=50}, false);
        pros::delay(500);
        drive(-127, 1300);
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
