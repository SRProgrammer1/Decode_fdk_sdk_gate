package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Ramp_Servo {
    private CRServo left_ramp_servo;
    private CRServo right_ramp_servo;
    private CRServo d_left_ramp_servo;
    private CRServo d_right_ramp_servo;



    public void init(HardwareMap hwMap) {
        left_ramp_servo = hwMap.get(CRServo.class, "ur_leftservo");
        right_ramp_servo = hwMap.get(CRServo.class, "ur_rightservo");
        d_left_ramp_servo = hwMap.get(CRServo.class, "d_lrservo");
        d_right_ramp_servo = hwMap.get(CRServo.class, "d_rrservo");

    }

    public void setServo_ramp(double power) {
        left_ramp_servo.setPower(power);
        right_ramp_servo.setPower(-power);
        d_left_ramp_servo.setPower(-power);
        d_right_ramp_servo.setPower(power);


    }
}
