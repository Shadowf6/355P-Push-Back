#include "main.h"

pros::MotorGroup leftMotors({-12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({8, -9, 10}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 15.0f, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(3);
pros::Rotation horizontalRotation(-6);
pros::Rotation verticalRotation(7);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_275, -4.5);
lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_275, -0.5);
lemlib::OdomSensors odometry(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);

lemlib::ControllerSettings lateralPID(10, 0, 10, 3, 1, 100, 3, 500, 0);
lemlib::ControllerSettings angularPID(4, 0, 11, 3, 1, 100, 3, 500, 0); 

lemlib::ExpoDriveCurve lateralCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve angularCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry, &lateralCurve, &angularCurve);

pros::Motor bottom(5);
pros::Motor basket(-16);
pros::Motor lift(17);
pros::Motor top(-15);

pros::adi::Pneumatics tongue('A', false);
pros::adi::Pneumatics elbowL('H', false);
pros::adi::Pneumatics elbowR('D', false); 

lv_obj_t *screen;

// pros::Distance distL(0);
// pros::Distance distR(0);
// pros::Distance distB(0);
// pros::Distance distT(0);

// Odometry odom(&imu, &verticalWheel, &horizontalWheel);
// Distance dist(&distL, &distR, &distB, &distT);
// MCL mcl(&odom, &dist, &chassis);

// float l0, l1 = 0.0f, r0, r1 = 0.0f;

void initialize() {
    chassis.calibrate(true);
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

    /*pros::Task odomUpdate([&]() {
        while (true) {
            odom.update();
            pros::delay(10);
        }
    });
    */

    /*pros::Task localization([&]() {
        while (true) {
            l0 = l1; l1 = (float)leftMotors.get_position(1);
            r0 = r1; r1 = (float)rightMotors.get_position(1);
            float encoderChange = (l1 - l0 + r1 - r0) / 2.0f;
            
            mcl.update(encoderChange, 500);

            pros::delay(10);
        }
    });*/
}

void disabled() {}

void competition_initialize() {
    horizontalRotation.reset_position();
    horizontalWheel.reset();
    verticalRotation.reset_position();
    verticalWheel.reset();
}

void autonomous() {
    imu.set_heading(0.0);
    chassis.setPose(0.0f, 0.0f, 0.0f);

    auto intake([&]() {basket.move(-127); bottom.move(127); lift.move(-127);});
    auto outtake([&]() {basket.move(127); bottom.move(-127); lift.move(127);});
    auto longGoal([&]() {basket.move(16); bottom.move(127); lift.move(127); top.move(127);});
    auto middleGoal([&]() {basket.move(16); bottom.move(127); lift.move(127); top.move(-114);});
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed); pros::delay(ms); chassis.tank(0, 0);});
    auto reset([&]() {basket.move(0); bottom.move(0); lift.move(0); top.move(0);});

    if (auton == 1) { // Left
        // Middle Goal
        intake();
        chassis.moveToPoint(-13, 30, 2000, {.maxSpeed=60}, false);
        chassis.moveToPoint(-9, 23, 750, {.forwards=false, .maxSpeed=60}, false);
        chassis.turnToHeading(40, 750, {}, false);
        chassis.moveToPoint(0, 36, 1500, {.maxSpeed=60}, false);
        chassis.turnToHeading(50, 750);
        middleGoal();
        pros::delay(3000);
        reset();

        // Match Load
        chassis.moveToPoint(-27.5, 12.5, 2250, {.forwards=false}, false);
        chassis.turnToHeading(178, 1000, {.maxSpeed=80}, false);
        intake();
        tongue.extend();
        drive(110, 1500);
        drive(-80, 200);
        pros::delay(750);
        
        // Long Goal
        chassis.moveToPoint(-32, 18, 1000, {.forwards=false, .maxSpeed=80}, false);
        longGoal();
    } else if (auton == 2) { // Right
        // Middle Goal
        intake();
        chassis.moveToPoint(12, 30, 2000, {.maxSpeed=50}, false);
        chassis.moveToPoint(9, 23, 750, {.forwards=false, .maxSpeed=60}, false);
        chassis.turnToHeading(-40, 750, {}, false);
        chassis.moveToPoint(-1, 33, 1500, {.maxSpeed=60}, false);
        chassis.turnToHeading(-48, 750);
        outtake();
        pros::delay(2000);
        reset();

        // Match Load
        chassis.moveToPoint(32, 9, 2000, {.forwards=false}, false);
        chassis.turnToHeading(-178, 1000, {.maxSpeed=80}, false);
        intake();
        tongue.extend();
        drive(110, 1500);
        drive(-80, 200);
        pros::delay(750);

        // Long Goal
        chassis.moveToPoint(29, 15, 1000, {.forwards=false, .maxSpeed=80}, false);
        longGoal();
    } else if (auton == 3) { // Skills 
        // Middle Goal
        intake();
        chassis.moveToPoint(-13, 30, 2000, {.maxSpeed=60}, false);
        chassis.moveToPoint(-9, 23, 750, {.forwards=false, .maxSpeed=60}, false);
        pros::delay(1000);
        chassis.turnToHeading(40, 750, {}, false);
        chassis.moveToPoint(0, 36, 1500, {.maxSpeed=60}, false);
        chassis.turnToHeading(50, 750);
        middleGoal();
        pros::delay(5000);
        reset();

        // Match Load
        chassis.moveToPoint(-27.5, 12.5, 3000, {.forwards=false}, false);
        chassis.turnToHeading(178, 1000, {.maxSpeed=80}, false);
        intake();
        tongue.extend();
        drive(110, 1500);       
        drive(-80, 200);       
        pros::delay(2000);
        drive(-80, 200);
        pros::delay(2000);
        
        // Long Goal
        chassis.moveToPoint(-32, 18, 1000, {.forwards=false, .maxSpeed=80}, false);
        longGoal();
        pros::delay(10000);
        reset();
        chassis.tank(100, 500);
        chassis.tank(-127, 1000);
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
            // Intake
            basket.move(-127);
            bottom.move(127);
            lift.move(-127);      
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            // Intake to top
            basket.move(-127);
            bottom.move(127);
            lift.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { 
            // Outtake
            basket.move(127);
            bottom.move(-114);
            lift.move(127);
            top.move(127); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { 
            // Long goal
            basket.move(16);
            bottom.move(127);
            lift.move(127);
            top.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { 
            // Middle goal
            basket.move(16);
            bottom.move(127);
            lift.move(127);
            top.move(-114);
        } else {
            basket.move(0);
            bottom.move(0);
            lift.move(0);
            top.move(0);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            // Tongue
            if (tongue.is_extended()) tongue.retract();
            else tongue.extend();
            pros::delay(250);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            // Left Elbow
            elbowR.retract();
            if (elbowL.is_extended()) elbowL.retract();
            else elbowL.extend();
            pros::delay(250);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            // Right Elbow
            elbowL.retract();
            if (elbowR.is_extended()) elbowR.retract();
            else elbowR.extend();
            pros::delay(250);
        }

        pros::delay(20);
    }
}
