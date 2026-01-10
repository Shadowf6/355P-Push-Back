#include "main.h"

pros::MotorGroup leftMotors({-8, 9, -10}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({16, -17, 18}, pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors, &rightMotors, 12, lemlib::Omniwheel::NEW_325, 450, 2);

lemlib::ControllerSettings lateralPID(6, 0, 5, 3, 1, 100, 3, 500, 0);
lemlib::ControllerSettings angularPID(5, 0, 20, 3, 1, 100, 3, 500, 0); 

pros::Imu imu(19);
pros::Rotation verticalRotation(15);
pros::Rotation horizontalRotation(-7);
lemlib::TrackingWheel verticalWheel(&verticalRotation, lemlib::Omniwheel::NEW_2, -0.5);
lemlib::TrackingWheel horizontalWheel(&horizontalRotation, lemlib::Omniwheel::NEW_275, 0.5);
lemlib::OdomSensors odometry(&verticalWheel, nullptr, &horizontalWheel, nullptr, &imu);
lemlib::Chassis chassis(drivetrain, lateralPID, angularPID, odometry);

pros::Motor intake(-20, pros::MotorGearset::blue);
pros::Motor score(-1, pros::MotorGearset::blue);

pros::adi::Pneumatics pivot('A', false);
pros::adi::Pneumatics tongue('C', false);
pros::adi::Pneumatics wing('B', false);

pros::Distance left(6);
pros::Distance right(14);
pros::Distance bottom(13);

lv_obj_t *screen;

void initialize() {                                                                                                                                                                                             
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    score.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    screen = lv_screen_active();
    createDisplay(screen);
    updateStatus(
        verticalRotation.is_installed() && horizontalRotation.is_installed(), 
        imu.is_installed(), 
        left.is_installed() && right.is_installed() && bottom.is_installed(), 
        intake.is_installed() && score.is_installed()
    );

    pros::Task display([&]() {
        while (true) {
            auto pose = chassis.getPose();
            updateCoords(pose.x, pose.y, pose.theta, 0.0f, 0.0f, 0.0f);

            pros::delay(30);
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
    auto in([&]() {intake.move(127); score.move_absolute(0.0, 600);});
    auto out([&](int ms) {intake.move(-100); pros::delay(ms); intake.move(0);});
    auto reset([&]() {intake.move(0); score.move(0); score.tare_position();});
    auto goal([&]() {intake.move(127); score.move(127);});
    
    auto drive([&](int speed, int ms) {chassis.tank(speed, speed, true); pros::delay(ms); chassis.tank(0, 0, true);});
    auto toWall([&](char side, float dist, float exit, int dir, int speed) {
        if (side == 'l') {
            if (dir == 1) while (inch(left.get()) <= dist - exit) { chassis.tank(speed, speed); pros::delay(30); }
            else if (dir == -1) while (inch(left.get()) >= dist + exit) { chassis.tank(-speed, -speed); pros::delay(30); }
        } else if (side == 'r') {
            if (dir == 1) while (inch(right.get()) <= dist - exit) { chassis.tank(speed, speed); pros::delay(30); }
            else if (dir == -1) while (inch(right.get()) >= dist + exit) { chassis.tank(-speed, -speed); pros::delay(30); }
        } else if (side == 'b') {
            if (dir == 1) while (inch(bottom.get()) >= dist + exit) { chassis.tank(-speed, -speed); pros::delay(30); }
            else if (dir == -1) while (inch(bottom.get()) <= dist - exit) { chassis.tank(speed, speed); pros::delay(30); }
        }
        chassis.tank(0, 0);
    });

    if (auton == 1) { // 4+3+3 (Right)
        // Match Load
        chassis.moveToPoint(0, 35.5, 1000, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(88, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(14, 35.5, 1000, {}, false);
        
        // Long Goal
        chassis.moveToPoint(-45, 35, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50); 
        goal(); 
        pros::delay(1250);
        tongue.retract();
        chassis.tank(0, 0);
        reset();
        chassis.setPose(-45, 34, 90);

        // Right Corner
        chassis.swingToHeading(185, DriveSide::RIGHT, 1000, {.maxSpeed=60}, false);
        in();
        drive(50, 400);
        pros::delay(100);

        // Left Corner
        chassis.turnToHeading(-175, 500, {}, false);
        chassis.moveToPoint(-45, -41, 1000, {.maxSpeed=80}, false);

        // Mid Goal
        chassis.turnToHeading(120, 500, {.maxSpeed=80}, false);
        reset();
        chassis.moveToPoint(-59, -26, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(135, 250, {}, false);
        pivot.extend();
        goal();
        pros::delay(750);
        pivot.retract();
        reset();
        chassis.setPose(-60, -27, 135);

        // Match Load
        chassis.moveToPoint(-25, -63.5, 1250, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(88, 500, {}, false);
        in();
        chassis.moveToPoint(-4, -63.5, 1000, {.maxSpeed=110}, false);

        // Long Goal
        chassis.moveToPoint(-50, -64, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
    } else if (auton == 2) { // 3+4 (Left)
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
        pros::delay(600);
        pivot.retract();
        reset();

        // Match Load
        chassis.moveToPoint(-34.25, 2, 2000, {.maxSpeed=80}, false); 
        chassis.turnToHeading(180, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(-34.5, -13.4, 1500, {.maxSpeed=60}, false);
        
        // Long Goal
        chassis.moveToPoint(-34.5, 30, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        goal();
    } else if (auton == 3) { // 7 (Right)
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
        out(100);
        goal();
    } else if (auton == 4) { // 4+W (Right)
        // Match Load
        chassis.moveToPoint(0, 35.5, 1000, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(88, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(14, 35.5, 1000, {}, false);
        
        // Long Goal
        chassis.moveToPoint(-45, 35, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50); 
        goal(); 
        pros::delay(1250);
        tongue.retract();
        chassis.tank(0, 0);
        reset();
        chassis.setPose(-45, 34, 90);
        wing.extend();
        wing.retract();

        // Wing
        chassis.moveToPoint(-35, 44, 1000, {.maxSpeed=80}, false);
        chassis.turnToHeading(90, 750, {}, false);
        chassis.moveToPoint(-64, 44, 20000000, {.forwards=false, .maxSpeed=60});
    } else if (auton == 5) { // 4+W (Left)
        // Match Load
        chassis.moveToPoint(0, 35.5, 1000, {.maxSpeed=100}, false);
        tongue.extend();
        chassis.turnToHeading(-88, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(-14, 35.5, 1000, {}, false);
        
        // Long Goal
        chassis.moveToPoint(45, 35, 750, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50); 
        goal(); 
        pros::delay(1250);
        tongue.retract();
        chassis.tank(0, 0);
        reset();
        chassis.setPose(45, 34, -90);
        wing.extend();
        wing.retract();

        // Wing
        chassis.moveToPoint(35, 44, 1000, {.maxSpeed=80}, false);
        chassis.turnToHeading(-90, 750, {}, false);
        chassis.moveToPoint(64, 44, 20000000, {.forwards=false, .maxSpeed=60});
    } else if (auton == 6) { // Skills
        // BR ML
        chassis.moveToPoint(0, 35.5, 1500, {.maxSpeed=80}, false);
        tongue.extend();
        chassis.turnToHeading(90, 750, {.maxSpeed=80}, false);
        in();
        chassis.moveToPoint(13.3, 36, 7000, {.maxSpeed=80}, false);

        // TR LG 
        reset();
        chassis.moveToPoint(-21, 48, 2000, {.forwards=false, .maxSpeed=80}, false);
        chassis.turnToHeading(86, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-71, 48, 2500, {.forwards=false, .maxSpeed=80}, false);
        chassis.moveToPoint(-90, 33, 1500, {.forwards=false, .maxSpeed=80}, false);  
        chassis.turnToHeading(-90, 750, {.maxSpeed=80}, false);
        chassis.moveToPoint(-70, 35, 1000, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(100);
        goal();
        pros::delay(3000);
        in();

        // TR ML
        chassis.moveToPoint(-118, 33.75, 6000, {.maxSpeed=60}, false);

        // TR LG
        chassis.moveToPoint(-70, 35, 1500, {.forwards=false, .maxSpeed=80}, false);
        chassis.tank(-50, -50);
        out(100);
        goal();
        pros::delay(3000);
        drive(50, 500);
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    
    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // Intake
            pivot.retract();
            intake.move(127);
            score.move_absolute(0.0, 600);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // Outtake
            intake.move(-100);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // Long Goal
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
            score.tare_position();
        }

        // Elbow
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) wing.extend();
        else wing.retract();

        // Tongue
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            if (tongue.is_extended()) tongue.retract();
            else tongue.extend();
            pros::delay(250);
        }
    
        pros::delay(20);
    }
}
