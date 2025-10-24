package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ServoBench {
    private CRServo kicker;

    public void init(HardwareMap hwMap) {
        kicker = hwMap.get(CRServo.class, "Kicker");
    }

    public void setServoRot(double power) {
        kicker.setPower(power);
    }
}
