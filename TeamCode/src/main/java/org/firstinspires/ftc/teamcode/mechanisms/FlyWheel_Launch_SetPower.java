package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlyWheel_Launch_SetPower {

    private DcMotor motor1, motor2;

    public void init(HardwareMap hwMap) {
        try {
            motor1 = hwMap.get(DcMotor.class, "fly_wheel_1");
            motor2 = hwMap.get(DcMotor.class, "fly_wheel_2");

            if (motor1 == null || motor2 == null) {
                throw new IllegalArgumentException("One or both flywheel motors not found in configuration!");
            }

            motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            motor1.setDirection(DcMotor.Direction.FORWARD);
            motor2.setDirection(DcMotor.Direction.REVERSE); // Opposite spin

        } catch (Exception e) {
            // Log the issue for debugging
            e.printStackTrace();
            motor1 = null;
            motor2 = null;
        }
    }

    public void setMotorSpeed(double speed1, double speed2) {
        if (motor1 != null && motor2 != null) {
            motor1.setPower(speed1);
            motor2.setPower(speed2);
        }
    }

    public boolean isInitialized() {
        return (motor1 != null && motor2 != null);
    }
}
