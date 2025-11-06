package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class MecanumDrive_Robot {

    private DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;

    public void init(HardwareMap hwMap) {
        frontLeftDrive = hwMap.get(DcMotor.class, "f_l_dr");
        backLeftDrive = hwMap.get(DcMotor.class, "b_l_dr");
        frontRightDrive = hwMap.get(DcMotor.class, "f_r_dr");
        backRightDrive = hwMap.get(DcMotor.class, "b_r_dr");


        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
/*
    public void drive_robot(double forward, double right, double rotate, double maxSpeed) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        //double maxSpeed = 1.0;  // make this slower for outreaches


        // This is needed to make sure we don't pass > 1.0 to any wheel
        // It allows us to keep all of the motors in proportion to what they should
        // be and not get clipped
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));


        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // We multiply by maxSpeed so that it can be set lower for outreaches
        // When a young child is driving the robot, we may not want to allow full
        // speed.
        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
 */

    public void drive_robot(double forward, double right, double rotate, double maxSpeed) {
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        // Normalize powers if any exceed 1.0
        double maxPower = Math.max(
                1.0,
                Math.max(Math.abs(frontLeftPower),
                        Math.max(Math.abs(frontRightPower),
                                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))))
        );

        // Apply power with scaling
        frontLeftDrive.setPower((frontLeftPower / maxPower) * maxSpeed);
        frontRightDrive.setPower((frontRightPower / maxPower) * maxSpeed);
        backLeftDrive.setPower((backLeftPower / maxPower) * maxSpeed);
        backRightDrive.setPower((backRightPower / maxPower) * maxSpeed);
    }

    public DcMotor getFrontLeft() { return frontLeftDrive; }
    public DcMotor getFrontRight() { return frontRightDrive; }
    public DcMotor getBackLeft() { return backLeftDrive; }
    public DcMotor getBackRight() { return backRightDrive; }

}
