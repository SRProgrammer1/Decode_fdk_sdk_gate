package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class FlyWheel_Launch_SetVelocity {

    private DcMotorEx motor1, motor2;
    private static final double TICKS_PER_REV = 28.0;

    public double getMotor1Velocity() { return motor1.getVelocity(); }
    public double getMotor2Velocity() { return motor2.getVelocity(); }

    public void init(HardwareMap hwMap) {
        motor1 = hwMap.get(DcMotorEx.class, "fly_wheel_1");
        motor2 = hwMap.get(DcMotorEx.class, "fly_wheel_2");

        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set directions opposite for mirror spinning
        motor1.setDirection(DcMotorEx.Direction.FORWARD);
        motor2.setDirection(DcMotorEx.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(0.05, 0.01, 0, 13.5);
        motor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        motor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    /** Set motor RPM (mirror image achieved by opposite directions) */
    public void setMotorRPM(double rpm) {
        double velocity = rpmToTicksPerSecond(rpm);
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity); // same velocity, direction handles mirroring
    }

    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }
}
