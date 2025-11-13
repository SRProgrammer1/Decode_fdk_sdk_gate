package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetVelocity;
import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetVelocity_Debug;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;

@TeleOp(name = "TeleOp_SetVelocity_Adjust", group = "TeleOp")
public class Teleop_SetVelocity_Adjust extends OpMode {

    ServoBench kicker = new ServoBench();
    MecanumDrive_Robot drive_r = new MecanumDrive_Robot();
    private FlyWheel_Launch_SetVelocity_Debug launch = new FlyWheel_Launch_SetVelocity_Debug();
    Ramp_Servo servo = new Ramp_Servo();
    intake_dcmotor intake_motor = new intake_dcmotor();
    boolean lastButtonState = false;
    boolean lastButtonState2 = false;
    double forward, right, rotate;
    double maxSpeed = 0.6;
    double start_stop = 0.0;
    double launcher_Speed = 0.0;

    boolean on_off = false;
    boolean on_off2 = false;

    int yPressedCount = 0;

    int bPressedCount = 0;

    @Override
    public void init() {
        drive_r.init(hardwareMap);
        launch.init(hardwareMap);
        servo.init(hardwareMap);
        intake_motor.init(hardwareMap);
        kicker.init(hardwareMap);
       // launcher_Speed = 2200;


        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean currentButtonState = gamepad2.a;
        boolean currentButtonState2 = gamepad2.x;

        forward = gamepad1.left_stick_y;
        right = gamepad1.right_stick_x;
        rotate = gamepad1.left_stick_x;


        drive_r.drive_robot(forward, right, rotate, maxSpeed);

        // Intake and ramp control

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
                kicker.setServoRot(0.25);
            } else {
                start_stop = 0.0;
                kicker.setServoRot(0.0);
            }
        }

        intake_motor.setMotorSpeed_intake(start_stop);
        servo.setServo_ramp(start_stop);

      /*  if(gamepad2.y) {
            yPressedCount = yPressedCount + 20;
        }
        if(gamepad2.b) {
            bPressedCount = bPressedCount + 20;
        }*/

        // Flywheel control
        if (gamepad2.right_bumper) {
            launcher_Speed = 2500;
        } else if (gamepad2.left_bumper) {
            launcher_Speed = 2100;
        } else if (gamepad2.right_trigger > 0.5) {
            launcher_Speed = 1500;
        } else if (gamepad2.left_trigger > 0.5) {
            launcher_Speed = 0.0;
        }
        launch.stabilizeVelocity(launcher_Speed);

       // launch.setMotorSpeed(left_motor, right_motor);



        // update for next loop
        lastButtonState = currentButtonState;
        lastButtonState2 = currentButtonState2;


        // Debug feedback

        telemetry.addData("Left Flywheel Power", launch.getMotor1Velocity());
        telemetry.addData("Right Flywheel Power", launch.getMotor2Velocity());
        telemetry.addData("Ramp/Intake Power", start_stop);
        telemetry.addData("Speed Mode", maxSpeed);
        telemetry.update();
    }
}
