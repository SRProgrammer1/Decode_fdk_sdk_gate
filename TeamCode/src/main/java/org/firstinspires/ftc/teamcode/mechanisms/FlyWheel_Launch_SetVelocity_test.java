package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


public class FlyWheel_Launch_SetVelocity_test {

    private DcMotorEx motor1, motor2;
    private static final double TICKS_PER_REV = 28.0;

    public double getMotor1Velocity() { return motor1.getVelocity(); }
    public double getMotor1Position() { return motor1.getCurrentPosition(); }
    public double getMotor2Velocity() { return motor2.getVelocity(); }

    public void init(HardwareMap hwMap) {
        motor1 = hwMap.get(DcMotorEx.class, "fly_wheel_1");
        motor2 = hwMap.get(DcMotorEx.class, "fly_wheel_2");

        //motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //motor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set directions opposite for mirror spinning
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        /* How to Fine-Tune
        Adjust F first:
        Too slow → increase F slightly (+0.5 each test)
        Overshooting → decrease F slightly (-0.5 each test)

        Adjust P if needed:
        Oscillations → lower P
        Slow response → increase P slightly */

        // 1300 f*1300

        PIDFCoefficients pidf = new PIDFCoefficients(2, 0.0, 2, 10);

        motor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        motor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
    }

    /** Set motor RPM (mirror image achieved by opposite directions) */
    public void setVelocityRPM(double rpm) {
        double velocity = rpmToTicksPerSecond(rpm);
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity); // same velocity, direction handles mirroring
    }

    private double rpmToTicksPerSecond(double rpm) {

        return (rpm * TICKS_PER_REV) / 60.0;
    }
}
