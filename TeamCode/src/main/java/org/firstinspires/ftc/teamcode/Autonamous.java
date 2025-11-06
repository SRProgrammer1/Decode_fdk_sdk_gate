package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetPower;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;
@Disabled

public class Autonamous extends OpMode {
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

    @Override
    public void init() {
        drive_r.init(hardwareMap);
        launch.init(hardwareMap);
        servo.init(hardwareMap);
        intake_motor.init(hardwareMap);
        kicker.init(hardwareMap);
    }

    @Override
    public void loop() {

        left_motor = 0.53;
        right_motor = 0.53;

        launch.setMotorSpeed(left_motor, right_motor);



        start_stop = 1.0;

        intake_motor.setMotorSpeed_intake(start_stop);
        servo.setServo_ramp(start_stop);

        drive_r.drive_robot(forward, right, rotate, maxSpeed);
    }
}
