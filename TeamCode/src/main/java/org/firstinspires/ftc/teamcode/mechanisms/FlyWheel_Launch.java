package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class FlyWheel_Launch {

    private DcMotor motor1, motor2;

    public void init(HardwareMap hwMap){
        motor1 = hwMap.get(DcMotor.class, "fly_wheel_1");
        motor2 = hwMap.get(DcMotor.class, "fly_wheel_2");
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setMotorSpeed(double speed1, double speed2) {
        motor1.setPower(speed1);
        motor2.setPower(speed2);
    }
}
