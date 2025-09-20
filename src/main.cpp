#include "main.h"

// Reminders:
// Virtual Skills Key
// Update Firmware
// Test Smart Field Control
// Autonomous Motor Functions

pros::MotorGroup leftMotors({-12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({8, -9, 10}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 15, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(3);
pros::Rotation horizontalRotation(-6);
pros::Rotation verticalRotation(7);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_275, -4.5);
lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_275, -0.5);
lemlib::OdomSensors odometry(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);
// Odometry odom(&imu, &verticalWheel, &horizontalWheel);

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

// pros::Distance distX(0);
// pros::Distance distY(0);

// float l0, l1 = 0.0f, r0, r1 = 0.0f;

void initialize() {
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    /*float initX = inch(distX.get_distance() + distXOffset);
    float initY = inch(distY.get_distance() + distYOffset);
    std::vector<float> init = {initX, initY, 0.0f};*/
    
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

    /*pros::Task juancarlos([&]() {
        while (true) {
            l0 = l1; l1 = (float)leftMotors.get_position(1);
            r0 = r1; r1 = (float)rightMotors.get_position(1);

            float encoderChange = (l1 - l0 + r1 - r0) / 2.0f;

            // mcl(&chassis, &distX, &distY, &imu, init, encoderChange, 500);

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
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);

    auto intake([&](){basket.move(-127); bottom.move(127); lift.move(-127);});
    auto outtake([&](){basket.move(127); bottom.move(-114); lift.move(127);});
    auto longGoal([&](){basket.move(16); bottom.move(127); lift.move(127); top.move(127);});
    auto middleGoal([&](){basket.move(16); bottom.move(127); lift.move(127); top.move(-114);});
    auto reset([&](){basket.move(0); bottom.move(0); lift.move(0); top.move(0);});

    //intake();

    if (auton == 1) { // Left
    } else if (auton == 2) { // Right
        intake();
        chassis.moveToPoint(12, 30, 2000, {.maxSpeed=50}, false);
        chassis.moveToPoint(8, 21, 750, {.forwards=false, .maxSpeed=60}, false);
        chassis.turnToHeading(-37, 750, {}, false);
        chassis.moveToPoint(-1, 32, 1500, {.maxSpeed=60}, false);
        chassis.turnToHeading(-35, 1000);
        outtake();
        chassis.waitUntilDone();
        reset();
    } else if (auton == 3) { // Skills
        
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        // Control Scheme 1 (Karl): 
        // Intake - L2
        // Outtake - L1
        // Long Goal - R2
        // Middle Goal - R1
        // Tongue - X
        // Elbow - LEFT/RIGHT

        // Control Scheme 2 (Shakey): 
        // Intake - L2
        // Outtake - R1
        // Long Goal - R2
        // Middle Goal - L1
        // Tongue - DOWN
        // Elbow - LEFT/RIGHT

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
            // Intake
            basket.move(-127);
            bottom.move(127);
            lift.move(-127);         
        } else if ((!skills && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) || (skills && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))) { 
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
        } else if ((!skills && controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) || (skills && controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1))) { 
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

        if ((!skills && controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) || (skills && controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN))) {
            // Tongue
            if (tongue.is_extended()) tongue.retract();
            else tongue.extend();
            pros::delay(500);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            // Left Elbow
            elbowR.retract();
            if (elbowL.is_extended()) elbowL.retract();
            else elbowL.extend();
            pros::delay(500);
        }

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            // Right Elbow
            elbowL.retract();
            if (elbowR.is_extended()) elbowR.retract();
            else elbowR.extend();
            pros::delay(500);
        }

        pros::delay(20);
    }
}
