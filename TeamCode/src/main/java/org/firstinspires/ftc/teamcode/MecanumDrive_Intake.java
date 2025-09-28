package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Field;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Servo_Intake;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench1;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

//@Disabled
@TeleOp
public class MecanumDrive_Intake extends OpMode {
    MecanumDrive_Robot drive_r = new MecanumDrive_Robot();
    // FlyWheel_Launch launch = new FlyWheel_Launch();
    Servo_Intake servo = new Servo_Intake();
    //private ElapsedTime runtime = new ElapsedTime();
    //boolean ServoOn = false;
    double forward, right, rotate;
    double maxSpeed = 0.5;
    double start_stop;

    @Override
    public void init() {
        drive_r.init(hardwareMap);
        // launch.init(hardwareMap);
        servo.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        //waitForStart();
        //runtime.reset();
    }

    @Override
    public void loop(){


        forward = -gamepad1.left_stick_y;
        right = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;
        if(gamepad1.a){
            maxSpeed = 1.0;
        } else if (gamepad1.b) {
            maxSpeed = 0.25;
        }
        drive_r.drive_robot(forward, right, rotate, maxSpeed);


        if (gamepad1.x) {
            start_stop = -1.0;
        }
        else if (gamepad1.y) {
            start_stop = 0.0;
        }
        servo.setServoRot(start_stop);


        /*    if (gamepad1.right_bumper) {
                launch.setMotorSpeed(-1.0, 1.0);
            } else if (gamepad1.right_trigger > 0.5) {
                launch.setMotorSpeed(-0.5, 0.5);

            } else if (gamepad1.left_trigger > 0.5) {
                launch.setMotorSpeed(0.0, 0.0);

            }*/



    }


}
