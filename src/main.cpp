#include "main.h"

pros::MotorGroup leftMotors({-0, 0, -0}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({0, -0, 0}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 14, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::ControllerSettings lateralPID(6, 0, 5, 3, 1, 100, 3, 500, 0);
lemlib::ControllerSettings angularPID(5, 0, 20, 3, 1, 100, 3, 500, 0); 

pros::Imu imu(0);
pros::Rotation verticalRotation(-0);
pros::Rotation horizontalRotation(0);
lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_2, -0.5);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_275, 1);
lemlib::OdomSensors odometry(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry);

pros::Motor intake(-0, pros::MotorGearset::blue);
pros::Motor score(-0, pros::MotorGearset::blue);

pros::adi::Pneumatics pivot('A', false);
pros::adi::Pneumatics tongue('B', false);
pros::adi::Pneumatics elbow('C', false);

pros::Distance left(0);
pros::Distance right(0);
pros::Distance bottom(0);

lv_obj_t *screen;

void initialize() {                                                                                                                                                                                             
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    score.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    lv_init();
    screen = lv_screen_active();
    createDisplay(screen);

    pros::Task display([&]() {
        while (true) {
            lv_timer_handler();

            auto pose = chassis.getPose();
            updateCoords(pose.x, pose.y, pose.theta);
            updateDist(inch(left.get()), inch(right.get()), inch(bottom.get()));

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
    imu.set_heading(0);
    chassis.setPose(0, 0, 0);
}

void autonomous() {
    auto in([&]() {intake.move(127); score.move(-40);});
    auto reset([&]() {intake.move(0); score.move(0);});
    auto goal([&]() {intake.move(127); score.move(127);});
    auto goon([&]() {intake.move(127); score.move(80);});
    
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed, true); pros::delay(ms); chassis.tank(0, 0, true);});
    auto toWall([&](char side, float dist, float exit, int dir, int speed) {
        if (side == 'l') {
            if (dir == 1) while (inch(left.get()) <= dist - exit) chassis.tank(speed, speed);
            else if (dir == -1) while (inch(left.get()) >= dist + exit) chassis.tank(-speed, -speed);
        } else if (side == 'r') {
            if (dir == 1) while (inch(right.get()) <= dist - exit) chassis.tank(speed, speed);
            else if (dir == -1) while (inch(right.get()) >= dist + exit) chassis.tank(-speed, -speed);
        } else if (side == 'b') {
            if (dir == 1) while (inch(bottom.get()) >= dist + exit) chassis.tank(-speed, -speed);
            else if (dir == -1) while (inch(bottom.get()) <= dist - exit) chassis.tank(speed, speed);
        }
        chassis.tank(0, 0);
    });

    if (auton == 1) { // Solo AWP
        // Match Load
        in();
        chassis.moveToPoint(0, 32, 1000, {.maxSpeed=80}, false);
        tongue.extend();   
        chassis.turnToHeading(90, 500, {}, false);
        chassis.moveToPoint(20, 35, 1000, {.maxSpeed=60}, false);
        
        // Long Goal
        chassis.moveToPoint(-40, 35, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50); 
        goal(); 
        pros::delay(2000);
        tongue.retract();
        chassis.tank(0, 0);
        in();

        // Right Corner
        chassis.swingToHeading(211, DriveSide::RIGHT, 1000, {.maxSpeed=60}, false);
        drive(50, 500);
        pros::delay(150);

        // Left Corner
        chassis.turnToHeading(180, 500, {}, false);
        chassis.moveToPoint(-8, -23, 1000, {.maxSpeed=80});
        pros::delay(800);
        tongue.extend();
        chassis.waitUntilDone();

        // Mid Goal
        chassis.turnToHeading(130, 500, {.maxSpeed=80}, false);
        chassis.moveToPoint(-18, -12, 500, {.forwards=false, .maxSpeed=80}, false);
        pivot.extend();
        goal();
        pros::delay(750);
        pivot.retract();
        in();

        // Match Load
        chassis.moveToPoint(15, -43, 1500, {.maxSpeed=80}, false);
        chassis.turnToHeading(90, 500, {}, false);
        chassis.moveToPoint(40, -43, 900, {.maxSpeed=50}, false);

        // Long Goal
        chassis.moveToPoint(-40, -52, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
    } else if (auton == 2) { // Left
        // Corner
        in();
        chassis.moveToPoint(-12, 26, 2500, {.maxSpeed=50}); 
        pros::delay(1200);
        tongue.extend(); 
        chassis.waitUntilDone();

        // Middle Goal
        chassis.turnToHeading(-135, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(3, 36.8, 1000, {.forwards=false, .maxSpeed=60}, false);
        pivot.extend();
        goal(); 
        pros::delay(800);
        pivot.retract();
        reset();

        // Match Load
        in();
        chassis.moveToPoint(-34.25, 2, 2000, {.maxSpeed=80}, false); 
        chassis.turnToHeading(180, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-34.5, -13.4, 1500, {.maxSpeed=60}, false);
        
        // Long Goal
        chassis.moveToPoint(-34.5, 30, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();

        // Wing
    } else if (auton == 3) { // Right
        // Corner
        in();
        chassis.moveToPoint(12, 26, 2500, {.maxSpeed=50}); 
        pros::delay(1200);
        tongue.extend();
        pros::delay(500); 
        chassis.waitUntilDone();

        // Match Load
        chassis.turnToHeading(142, 1000, {.maxSpeed=80}, false);
        chassis.moveToPoint(37.5, 2, 2000, {.maxSpeed=80}, false);
        chassis.turnToHeading(180, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(38, -12.7, 1400, {.maxSpeed=60}, false);
        
        // Long Goal
        chassis.moveToPoint(38.8, 30, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
        pros::delay(1000);
        intake.move(-127);
        pros::delay(250);
        goon();
        pros::delay(4000);
        
        // Wing
        drive(50, 150);
        pros::delay(500);
        drive(-127, 350);
    } else if (auton == 4) { // Skills
        // BR ML
        in();
        elbow.extend();
        chassis.moveToPoint(0, 34.25, 1500, {.maxSpeed=80}, false);
        tongue.extend();
        chassis.turnToHeading(90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(12.9, 37, 1500, {.maxSpeed=60}, false);
        chassis.moveToPoint(0, 37, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.moveToPoint(12.9, 37, 1500, {.maxSpeed=60}, false);

        // TR LG 
        chassis.moveToPoint(-21, 48, 2000, {.forwards=false, .maxSpeed=80}, false);
        reset();
        chassis.turnToHeading(86, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-71, 48, 2500, {.forwards=false, .maxSpeed=80}, false);
        chassis.moveToPoint(-90, 28, 1500, {.forwards=false, .maxSpeed=80}, false);  
        chassis.turnToHeading(-90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-70, 31, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
        pros::delay(3000);
        in();

        // TR ML
        chassis.moveToPoint(-106.65, 29.5, 2500, {.maxSpeed=60}, false);
        chassis.moveToPoint(-90, 30, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.moveToPoint(-106.65, 29.5, 1500, {.maxSpeed=60}, false);

        // TR LG
        chassis.moveToPoint(-70, 31, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
        pros::delay(3000);
        in();
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // Intake
            pivot.retract();
            intake.move(127);
            score.move(-40); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // Outtake
            intake.move(-60);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {  // Long Goal
            pivot.retract();
            intake.move(127);
            score.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) { // Middle Goal 
            pivot.extend();
            intake.move(127);
            score.move(127);
        } else { // Reset
            intake.move(0);
            score.move(0);
        }

        // Elbow
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) elbow.extend();
        else elbow.retract();

        // Tongue
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            if (tongue.is_extended()) tongue.retract();
            else tongue.extend();
            pros::delay(250);
        }
    
        pros::delay(20);
    }
}
