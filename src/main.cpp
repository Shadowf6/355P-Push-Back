#include "main.h"

pros::adi::Pneumatics wing('E', true);

lv_obj_t *screen;


void initialize() {                                                                                                                                                                                       
    chassis.calibrate();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    score.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    lv_init();
    screen = lv_screen_active();
    createDisplay(screen);
    setStatus(
        horizontalRotation.is_installed(), // Rotation sensor
        imu.is_installed(), // Inertial sensor
        left.is_installed() && right.is_installed(), // Distance sensors
        intake.is_installed() && score.is_installed() // Intake motors
    );

    pros::Task stick([&]() {
        while (true) {
            if (state == 0) stickMid();
            else if (state == 1) stickUp();
            else stickDown();

            pros::delay(10);
        }
    });

    pros::Task display([&]() {
        while (true) {
            if (driverControl && match) return;

            auto pose = chassis.getPose();
            updateCoords(pose.x, pose.y, pose.theta, inch(left.get()), inch(right.get()), 0, 0);

            pros::delay(100);
        }
    });
}

void disabled() {
    match = true;
}

void competition_initialize() {
    horizontalRotation.reset_position();
    horizontalWheel.reset();
    imu.set_heading(0);
    chassis.setPose(0, 0, 0);
    score.tare_position();
}

void autonomous() {
    wing.retract();
    if (auton == 1) { // 6 Right
        right6();
    } else if (auton == 2) { // 6 Left
        left6();
    } else if (auton == 3) { // 4 Right
        right4();
        rightWing();
    } else if (auton == 4) { // 4 Left
        left4();
        leftWing();
    } else if (auton == 5) { // Right Split
        rightSplit();
    } else if (auton == 6) { // Left Split
        leftSplit();
    } else if (auton == 7) { // Skills
        autonSkills();
    } else if (auton == 8) { // AWP Wing
        awpWing();
    } else if (auton == 9) { // Move Forward
        chassis.tank(50, 50);
        pros::delay(300);
        chassis.tank(0, 0);
    }
}

void opcontrol() {
    pros::Controller controller(pros::E_CONTROLLER_MASTER);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    driverControl = true;
    state = 2;
    bump.retract();

    while (true) {
        chassis.tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y), controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) { // Score
            hood.extend();
            if (!lift.is_extended()) bump.extend();
            intake.move(127);
            state = (int)lift.is_extended();
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { // Intake
            hood.retract();
            bump.retract();
            intake.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) { // Outtake
            intake.move(-127);
        } else { // Reset
            intake.brake();
            state = 2;
        }

        // Wing
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) wing.extend();
        else wing.retract();

        // Lift
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            if (lift.is_extended()) {lift.retract(); bump.retract();}
            else lift.extend();
        }
        
        // Hood
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            if (hood.is_extended()) hood.retract();
            else {hood.extend(); tongue.retract();}
        }

        // Tongue
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            if (!tongue.is_extended()) {tongue.extend(); hood.retract(); }
            else tongue.retract();
        }

        // Auton Testing
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && !match) {
            competition_initialize();
            pros::delay(3000);
            autonomous();
        }
    
        pros::delay(10);
    }
}
