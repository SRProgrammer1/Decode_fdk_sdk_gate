package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetPower;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;

@TeleOp(name = "TeleOp", group = "TeleOp")
public class Teleop_Intake_Ramp_Launch extends OpMode {

    ServoBench kicker = new ServoBench();
    MecanumDrive_Robot drive_r = new MecanumDrive_Robot();
    FlyWheel_Launch_SetPower launch = new FlyWheel_Launch_SetPower();
    Ramp_Servo servo = new Ramp_Servo();
    intake_dcmotor intake_motor = new intake_dcmotor();
    boolean lastButtonState = false;
    boolean lastButtonState2 = false;
    double forward, right, rotate;
    double maxSpeed = 0.5;
    double start_stop = 0.0;
    double left_motor = 0.0, right_motor = 0.0;

    boolean on_off = false;
    boolean on_off2 = false;

    @Override
    public void init() {
        drive_r.init(hardwareMap);
        launch.init(hardwareMap);
        servo.init(hardwareMap);
        intake_motor.init(hardwareMap);
        kicker.init(hardwareMap);


        telemetry.addData("Status", "Initialized");
        if (!launch.isInitialized()) {
            telemetry.addData("Warning", "Flywheel motors not found! Check configuration names.");
        }
        telemetry.update();
    }

    @Override
    public void loop() {
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

        // Flywheel control
        if (gamepad2.right_bumper) {
            left_motor = 0.44;
            right_motor = 0.44;
        } else if (gamepad2.right_trigger > 0.5) {
            left_motor = 0.40;
            right_motor = 0.40;
        } else if (gamepad2.left_trigger > 0.5) {
            left_motor = 0.0;
            right_motor = 0.0;
        }

        launch.setMotorSpeed(left_motor, right_motor);



        // update for next loop
        lastButtonState = currentButtonState;
        lastButtonState2 = currentButtonState2;


        // Debug feedback
        telemetry.addData("Left Flywheel Power", left_motor);
        telemetry.addData("Right Flywheel Power", right_motor);
        telemetry.addData("Ramp/Intake Power", start_stop);
        telemetry.addData("Speed Mode", maxSpeed);
        telemetry.update();
    }
}
