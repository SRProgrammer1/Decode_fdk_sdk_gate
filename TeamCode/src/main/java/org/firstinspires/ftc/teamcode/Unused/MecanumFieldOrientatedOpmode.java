package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Field;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;

@TeleOp
@Disabled
public class MecanumFieldOrientatedOpmode extends OpMode {

    MecanumDrive_Field drive = new MecanumDrive_Field();
    Ramp_Servo servo = new Ramp_Servo();


    double forward, right, rotate;
    double maxSpeed = 0.5;
    double start_stop;

    @Override
    public void init() {
        drive.init(hardwareMap);
        servo.init(hardwareMap);


    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        right = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        if(gamepad1.a){
            maxSpeed = 1.0;
        } else if (gamepad1.b) {
            maxSpeed = 0.25;
        }
        drive.driveFieldRelative(forward,right,rotate,maxSpeed);

        if (gamepad1.x) {
            start_stop = -1.0;
        }
        else if (gamepad1.y) {
            start_stop = 0.0;
        }
        servo.setServo_ramp(start_stop);
    }

}
