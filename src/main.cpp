#include "main.h"

pros::MotorGroup leftMotors({-1, 13, -12}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({8, -9, 10}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 15, lemlib::Omniwheel::NEW_325, 450, 2);

pros::Imu imu(0);
pros::Rotation horizontalRotation(0);
pros::Rotation verticalRotation(0);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_275, 0);
lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_275, 0);
lemlib::OdomSensors odometry(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);

lemlib::ControllerSettings lateralPID(10, 0, 10, 3, 1, 100, 3, 500, 0);
lemlib::ControllerSettings angularPID(4, 0, 11, 3, 1, 100, 3, 500, 0); 

lemlib::ExpoDriveCurve lateralCurve(3, 10, 1.019);
lemlib::ExpoDriveCurve angularCurve(3, 10, 1.019);

lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry, &lateralCurve, &angularCurve);

pros::Motor intake(15, pros::MotorGearset::blue);
pros::Motor score(-2, pros::MotorGearset::blue);

pros::adi::Pneumatics pivot('A', false);
pros::adi::Pneumatics elbow('B', false);
pros::adi::Pneumatics tongue('C', false);

lv_obj_t *screen;

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
}

void disabled() {}

void competition_initialize() {
    imu.tare_heading();
    horizontalRotation.reset_position();
    horizontalWheel.reset();
    verticalRotation.reset_position();
    verticalWheel.reset();
}

void autonomous() {
    chassis.setPose(0.0f, 0.0f, 0.0f);

    auto in([&]() {intake.move(127); score.move(-40);});
    auto goal([&]() {score.move(127);});
    auto reset([&]() {intake.move(0); score.move(0);});
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed); pros::delay(ms); chassis.tank(0, 0);});

    if (auton == 1) {
        // Left
    } else if (auton == 2) { 
        // Right
    } else if (auton == 3) { 
        // Skills 
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
