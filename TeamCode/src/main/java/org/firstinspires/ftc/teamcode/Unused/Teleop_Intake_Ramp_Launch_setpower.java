package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetPower;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp(name = "TeleOp_Launch_Adjust_tested_lastMatch", group = "TeleOp")
@Disabled
public class Teleop_Intake_Ramp_Launch_setpower extends OpMode {

    ServoBench kicker = new ServoBench();
    MecanumDrive_Robot drive_r = new MecanumDrive_Robot();
    FlyWheel_Launch_SetPower flywheel = new FlyWheel_Launch_SetPower();
    Ramp_Servo servo = new Ramp_Servo();
    intake_dcmotor intake_motor = new intake_dcmotor();
    boolean lastButtonState = false;
    boolean lastButtonState2 = false;
    double forward, right, rotate;
    double maxSpeed = 0.6;
    double start_stop = 0.0;
    double left_motor = 0.0, right_motor = 0.0;

    boolean on_off = false;
    boolean on_off2 = false;


    double voltage = 0;
    private VoltageSensor batteryVoltageSensor;
    double near_launch_power = 0.0;
    double med_launch_power = 0.0;
    double far_launch_power = 0.0;
    double Target_RPM = 0;
    boolean yWasPressed = false;
    boolean bWasPressed = false;

    private void rpmForHighVoltage() {
        if(voltage > 13.5){
            near_launch_power = 0.38;
            med_launch_power = 0.42;
            far_launch_power =0.46;

            //flywheel.setMotorSpeed(0.44, 0.44);
        }
        else if ((voltage > 13.0) && (voltage <= 13.5)){
            near_launch_power = 0.39;
            med_launch_power = 0.43;
            far_launch_power =0.48;
           // flywheel.setMotorSpeed(0.47, 0.47);
        } else if ((voltage > 12.5) && (voltage <= 13.0)){
            near_launch_power = 0.40;
            med_launch_power = 0.45;
            far_launch_power =0.49;
            //flywheel.setMotorSpeed(0.49, 0.49);
        }
        else{
            near_launch_power = 0.43;
            med_launch_power = 0.46;
            far_launch_power = 0.50;
            //flywheel.setMotorSpeed(0.50, 0.50);
        }
    }

    @Override
    public void init() {
        drive_r.init(hardwareMap);
        flywheel.init(hardwareMap);
        servo.init(hardwareMap);
        intake_motor.init(hardwareMap);
        kicker.init(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = batteryVoltageSensor.getVoltage();
        rpmForHighVoltage();
        telemetry.addData("Battery Voltage", "%.2f V", voltage);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        voltage = batteryVoltageSensor.getVoltage();

        rpmForHighVoltage();
        boolean currentButtonState = gamepad2.a;
        boolean currentButtonState2 = gamepad2.x;

        forward = gamepad1.left_stick_y;
        right = gamepad1.right_stick_x;
        rotate = gamepad1.left_stick_x;

        // Speed mode switch
   /*     if (gamepad1.a) {
            maxSpeed = 1.0;
        } else if (gamepad1.b) {
            maxSpeed = 0.25;
        }*/

        drive_r.drive_robot(forward, right, rotate, maxSpeed);

        // Intake and ramp control
        //if (gamepad2.x) {
          //  start_stop = 1.0;
        //} else if (gamepad2.y) {
          //  start_stop = 0.0;
        //}

        if(currentButtonState2 && !lastButtonState2) {
            on_off2 = !on_off2;

            if (on_off2) {
                start_stop = 1.0;
            }
            else {
                start_stop = 0.0;
                kicker.setServoRot(0.0);
            }
        }

        if (currentButtonState && !lastButtonState) {
            on_off = !on_off; // toggle state

            if (on_off) {
                start_stop = 1.0;
                kicker.setServoRot(1.0);
            } else {
                start_stop = 0.0;
                kicker.setServoRot(0.0);
            }
        }

        intake_motor.setMotorSpeed_intake(start_stop);
        servo.setServo_ramp(start_stop);


        if (gamepad1.y && !yWasPressed) {
            Target_RPM += 0.01;
        }
        yWasPressed = gamepad1.y;

// B = -20 RPM once
        if (gamepad1.b && !bWasPressed) {
            Target_RPM -= 0.01;
        }
        bWasPressed = gamepad1.b;

        // Flywheel control
        if (gamepad2.right_bumper) {


            Target_RPM = far_launch_power;
        } else if (gamepad2.right_trigger > 0.5) {
            Target_RPM = med_launch_power;


        } else if (gamepad2.left_bumper) {
            Target_RPM = near_launch_power;


        } else if (gamepad2.left_trigger > 0.5) {
            Target_RPM = 0;
        }

        flywheel.setMotorSpeed(Target_RPM, Target_RPM);

        // update for next loop
        lastButtonState = currentButtonState;
        lastButtonState2 = currentButtonState2;


        // Debug feedback
        telemetry.addData("Battery Voltage", "%.2f V", voltage);
        telemetry.addData("Flywheel Power", Target_RPM);
        telemetry.addData("Ramp/Intake Power", start_stop);
        telemetry.addData("Speed Mode", maxSpeed);
        telemetry.update();
    }
}
