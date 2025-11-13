package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class FlyWheel_Launch_SetVelocity_Debug {

    private DcMotorEx motor1, motor2;
    private VoltageSensor batteryVoltageSensor;

    private static final double TICKS_PER_REV = 28.0;
    private static final double GAIN = 0.05; // correction strength (0.03–0.1 recommended)

    private double correctedRPM = 0.0;

    public double getMotor1Velocity() { return motor1.getVelocity(); }
    public double getMotor2Velocity() { return motor2.getVelocity(); }
    public double getTicksPerRev() { return TICKS_PER_REV; }

    public void init(HardwareMap hwMap) {
        motor1 = hwMap.get(DcMotorEx.class, "fly_wheel_1");
        motor2 = hwMap.get(DcMotorEx.class, "fly_wheel_2");

        motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Set directions opposite for mirror spinning
        motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Base tuning (you already found this works well)
        PIDFCoefficients pidf = new PIDFCoefficients(8.0, 0.0, 1.0, 0.0);
        motor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        motor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        // Get first available voltage sensor
        batteryVoltageSensor = hwMap.voltageSensor.iterator().next();
    }

    /** Convert RPM to ticks per second for motor.setVelocity() */
    private double rpmToTicksPerSecond(double rpm) {
        return (rpm * TICKS_PER_REV) / 60.0;
    }

    /** Simple function to get battery voltage */
    public double getBatteryVoltage() {
        return batteryVoltageSensor.getVoltage();
    }

    /** Normal direct RPM set (no feedback correction) */
    public void setVelocityRPM(double rpm) {
        double velocity = rpmToTicksPerSecond(rpm);
        motor1.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }

    /**
     * Feedback-corrected RPM setter.
     * Adjusts RPM target dynamically based on difference between actual and target RPM.
     */
    public void stabilizeVelocity(double targetRPM) {
        double actualRPM = Math.abs(getMotor1Velocity() / TICKS_PER_REV * 60.0);
        double error = targetRPM - actualRPM;

        // Outer-loop correction (proportional)
        correctedRPM += error * GAIN;

        // Clamp to safe range
        correctedRPM = Math.max(0, Math.min(correctedRPM, 6000));

        // Send corrected RPM to motors
        setVelocityRPM(correctedRPM);
    }
}
