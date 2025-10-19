package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.Kicker;

@TeleOp
@Disabled
public class ServoTest extends OpMode {

    Kicker servo = new Kicker();

    @Override
    public void init() {
        servo.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            servo.setServoPos(0.4);
        }

        else {
            servo.setServoPos(0.6);
        }
    }
}
