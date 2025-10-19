package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;

//@Disabled
@TeleOp
public class Intake_ServoRamp extends OpMode {

    Ramp_Servo servo = new Ramp_Servo();
    intake_dcmotor intake_motor = new intake_dcmotor();
    double start_stop;

    @Override
    public void init() {
        servo.init(hardwareMap);
        intake_motor.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop(){

        if (gamepad1.x) {
            start_stop = 1.0;
        }
        else if (gamepad1.y) {
            start_stop = 0.0;
        }
        intake_motor.setMotorSpeed_intake(start_stop);
        servo.setServo_ramp(start_stop);

    }


}
