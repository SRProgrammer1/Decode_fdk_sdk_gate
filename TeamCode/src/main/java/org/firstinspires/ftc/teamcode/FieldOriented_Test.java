package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


@TeleOp(name="Field Oriented Test Drive", group="Linear Opmode")
public class FieldOriented_Test extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    @Override
    public void runOpMode() {
        // Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "f_l_dr");
        frontRight = hardwareMap.get(DcMotor.class, "f_r_dr");
        backLeft   = hardwareMap.get(DcMotor.class, "b_l_dr");
        backRight  = hardwareMap.get(DcMotor.class, "b_r_dr");

        // Reverse motors as needed for your drivetrain
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Configure IMU with hub orientation
        imu = hardwareMap.get(IMU.class, "imu");

        // Hub mounted backwards (logo facing back), USB to the RIGHT
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT);

        IMU.Parameters imuParams = new IMU.Parameters(orientationOnRobot);
        imu.initialize(imuParams);

        telemetry.addLine("IMU initialized. Ready to start.");
        telemetry.update();

        waitForStart();
        imu.resetYaw();


        while (opModeIsActive()) {
            // Read joystick values
            double y = -gamepad1.left_stick_y; // forward/back
            double x = gamepad1.left_stick_x;  // strafe
            double rx = gamepad1.right_stick_x; // rotation

            // Get robot heading from IMU
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Field-oriented transform
            //double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            //double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);


            // Normalize the power values
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower  = (rotY - rotX + rx) / denominator;
            double frontRightPower= (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            telemetry.addData("Heading (deg)", Math.toDegrees(botHeading));
            telemetry.update();
        }
    }
}