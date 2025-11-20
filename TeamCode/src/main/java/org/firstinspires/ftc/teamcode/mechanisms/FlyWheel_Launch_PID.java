package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlyWheel_Launch_PID {
    private DcMotorEx flywheel1;
    private DcMotorEx flywheel2;

    // Target velocity in ticks per second
    private double targetTPS = 0;

    // PIDF coefficients (tune these!)
    private double Kp = 0.0005; // Proportional
    private double Ki = 0.00001; // Integral
    private double Kd = 0;       // Derivative
    private double Kf = 1.0;     // Feedforward

    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime = 0;

    public void init(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "fly_wheel_1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "fly_wheel_2");

        flywheel1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheel1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotor.Direction.REVERSE);

        lastTime = System.currentTimeMillis();
    }

    public void setVelocityTPS(double targetTPS) {
        this.targetTPS = targetTPS;

        double currentTPS = flywheel1.getVelocity(); // Current velocity of motor1
        long now = System.currentTimeMillis();
        double deltaTime = (now - lastTime) / 1000.0; // in seconds
        lastTime = now;

        double error = targetTPS - currentTPS;
        integralSum += error * deltaTime;
        double derivative = (error - lastError) / deltaTime;
        lastError = error;

        // PIDF output
        double power = Kp * error + Ki * integralSum + Kd * derivative + Kf * (targetTPS / 1000.0);
        power = Math.max(0, Math.min(1, power)); // Clamp 0-1

        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    public double getMotor1Velocity() {
        return flywheel1.getVelocity();
    }

    public double getMotor2Velocity() {
        return flywheel2.getVelocity();
    }
}
