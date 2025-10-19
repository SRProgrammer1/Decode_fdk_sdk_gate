package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker {
    private Servo kicker;

    public void init(HardwareMap hwMap) {
        kicker = hwMap.get(Servo.class, "Kicker");
    }

    public void setServoPos(double angle) {
        kicker.setPosition(angle);
    }
}
