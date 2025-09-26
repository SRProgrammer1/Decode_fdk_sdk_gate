package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TestBench1 {
    private DcMotor motor;

    public void init(HardwareMap hwMap){
        motor = hwMap.get(DcMotor.class, "RightDrive");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setMotorSpeed(double speed) {
        motor.setPower(speed);
    }
}
