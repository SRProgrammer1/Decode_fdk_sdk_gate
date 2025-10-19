package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake_dcmotor {
    private DcMotor motor;

    public void init(HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, "intake_motor");
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setDirection(DcMotor.Direction.REVERSE);

    }
    public void setMotorSpeed_intake(double speed) {

        motor.setPower(speed);
    }
}
