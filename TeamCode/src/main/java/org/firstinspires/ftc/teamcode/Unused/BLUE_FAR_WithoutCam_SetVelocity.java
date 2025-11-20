package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetVelocity_test;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;

@Autonomous(name = "BLUE_FAR_WithoutCam_SetVelocity", group = "Auto")
@Disabled
public class BLUE_FAR_WithoutCam_SetVelocity extends LinearOpMode {

    private MecanumDrive_Robot drive = new MecanumDrive_Robot();
    private Ramp_Servo servo = new Ramp_Servo();
    double targetRPM = 2200;   // Launch power
    private FlyWheel_Launch_SetVelocity_test flywheel = new FlyWheel_Launch_SetVelocity_test();
    private intake_dcmotor intake = new intake_dcmotor();
    private ServoBench kicker = new ServoBench();

    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3.0;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
    static final double ROBOT_TRACK_WIDTH_INCHES = 15.0;


    @Override
    public void runOpMode() {

        drive.init(hardwareMap);
        flywheel.init(hardwareMap);
        intake.init(hardwareMap);
        kicker.init(hardwareMap);
        servo.init(hardwareMap);

        telemetry.addLine("Initialized. Waiting for start...");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            // === Step 1: Drive forward 72 inches (was backward) ===
           // driveDistance(-42, 0.5);
            //sleep(500);

            // === Step 2: Turn left 45 degrees (was right) ===


            // === Step 3: Start flywheels ===
            telemetry.addLine("Starting flywheels...");
            telemetry.update();
            flywheel.setVelocityRPM(targetRPM);
            sleep(4000);

            // === Step 4: Start intake + kicker + ramp ===
            telemetry.addLine("Starting intake and kicker...");
            telemetry.update();
            //Launch
            intake.setMotorSpeed_intake(1.0);
            kicker.setServoRot(0.25);
            servo.setServo_ramp(1.0);
            sleep(8000);

            // === Step 5: Stop all mechanisms ===
           // flywheel.setMotorSpeed(0.0, 0.0);
            intake.setMotorSpeed_intake(0);
            kicker.setServoRot(0.0);
            servo.setServo_ramp(0.0);

            telemetry.addLine("Shooting complete. Moving backward...");
            telemetry.update();
            sleep(500);
            //Drive Forward
            driveDistance(12, 0.4);


            turnDegreesLeft(280, 0.4);
            sleep(500);
            strafeDegreesRight(4,0.4);
            sleep(200);
            // === Step 6: Drive backward 24 inches (was forward) ===
            intake.setMotorSpeed_intake(1.0);
            servo.setServo_ramp(1.0);
            driveDistance(-26, 0.25);
            sleep(500);
            //driveDistance(20, 0.4);

            driveDistance(26, 0.4);
            turnDegreesLeft(120, 0.4);
            driveDistance(-13, 0.4);



            servo.setServo_ramp(1.0);
            intake.setMotorSpeed_intake(0.25);
            kicker.setServoRot(1.0);
            sleep(5000);

            driveDistance(14, 0.4);

            //Stop all mechanisms
            servo.setServo_ramp(0.0);
            intake.setMotorSpeed_intake(0.0);
            kicker.setServoRot(0.0);
            flywheel.setVelocityRPM(0);

            telemetry.addLine("Autonomous routine complete!");
            telemetry.update();
        }
    }

    // --- Helper methods using MecanumDrive_Robot ---

    private void driveDistance(double inches, double speed) {
        int targetTicks = (int) (Math.abs(inches) * TICKS_PER_INCH);

        // Reset encoders
        drive.getFrontLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.getFrontLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double direction = inches > 0 ? 1.0 : -1.0;

        int avgTicks;
        do {
            drive.drive_robot(direction * 1.0, 0, 0, speed);
            avgTicks = (Math.abs(drive.getFrontLeft().getCurrentPosition())
                    + Math.abs(drive.getFrontRight().getCurrentPosition())
                    + Math.abs(drive.getBackLeft().getCurrentPosition())
                    + Math.abs(drive.getBackRight().getCurrentPosition())) / 4;
            telemetry.addData("Driving ticks", "%d/%d", avgTicks, targetTicks);
            telemetry.update();
        } while (opModeIsActive() && avgTicks < targetTicks);

        drive.drive_robot(0, 0, 0, 0);
    }
    private void turnDegreesLeft(double degrees, double speed) {
        double turnCircumference = Math.PI * ROBOT_TRACK_WIDTH_INCHES;
        double turnDistance = (degrees / 360.0) * turnCircumference;
        int targetTicks = (int) (turnDistance * TICKS_PER_INCH);

        // Reset encoders
        drive.getFrontLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.getFrontLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int avgTicks;
        do {
            drive.drive_robot(0, -1.0, 0, speed); // rotate left (negative rotation)
            avgTicks = (Math.abs(drive.getFrontLeft().getCurrentPosition())
                    + Math.abs(drive.getFrontRight().getCurrentPosition())
                    + Math.abs(drive.getBackLeft().getCurrentPosition())
                    + Math.abs(drive.getBackRight().getCurrentPosition())) / 4;
            telemetry.addData("Turning ticks", "%d/%d", avgTicks, targetTicks);
            telemetry.update();
        } while (opModeIsActive() && avgTicks < targetTicks);

        drive.drive_robot(0, 0, 0, 0);
    }
    private void turnDegreesRight(double degrees, double speed) {
        double turnCircumference = Math.PI * ROBOT_TRACK_WIDTH_INCHES;
        double turnDistance = (degrees / 360.0) * turnCircumference;
        int targetTicks = (int) (turnDistance * TICKS_PER_INCH);

        // Reset encoders
        drive.getFrontLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.getFrontLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int avgTicks;
        do {
            drive.drive_robot(0, 1.0, 0, speed); // rotate left (negative rotation)
            avgTicks = (Math.abs(drive.getFrontLeft().getCurrentPosition())
                    + Math.abs(drive.getFrontRight().getCurrentPosition())
                    + Math.abs(drive.getBackLeft().getCurrentPosition())
                    + Math.abs(drive.getBackRight().getCurrentPosition())) / 4;
            telemetry.addData("Turning ticks", "%d/%d", avgTicks, targetTicks);
            telemetry.update();
        } while (opModeIsActive() && avgTicks < targetTicks);

        drive.drive_robot(0, 0, 0, 0);
    }
    private void strafeDegreesRight(double inches, double speed) {
        int targetTicks = (int) (Math.abs(inches) * TICKS_PER_INCH);

        // Reset encoders
        drive.getFrontLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.getFrontLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double direction = inches > 0 ? 1.0 : -1.0;

        int avgTicks;
        do {
            drive.drive_robot(0, 0, 1, speed);
            avgTicks = (Math.abs(drive.getFrontLeft().getCurrentPosition())
                    + Math.abs(drive.getFrontRight().getCurrentPosition())
                    + Math.abs(drive.getBackLeft().getCurrentPosition())
                    + Math.abs(drive.getBackRight().getCurrentPosition())) / 4;
            telemetry.addData("Driving ticks", "%d/%d", avgTicks, targetTicks);
            telemetry.update();
        } while (opModeIsActive() && avgTicks < targetTicks);

        drive.drive_robot(0, 0, 0, 0);
    }

    private void strafeDegreesLeft(double inches, double speed) {
        int targetTicks = (int) (Math.abs(inches) * TICKS_PER_INCH);

        // Reset encoders
        drive.getFrontLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.getFrontLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getFrontRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackLeft().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackRight().setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double direction = inches > 0 ? 1.0 : -1.0;

        int avgTicks;
        do {
            drive.drive_robot(0, 0, -1, speed);
            avgTicks = (Math.abs(drive.getFrontLeft().getCurrentPosition())
                    + Math.abs(drive.getFrontRight().getCurrentPosition())
                    + Math.abs(drive.getBackLeft().getCurrentPosition())
                    + Math.abs(drive.getBackRight().getCurrentPosition())) / 4;
            telemetry.addData("Driving ticks", "%d/%d", avgTicks, targetTicks);
            telemetry.update();
        } while (opModeIsActive() && avgTicks < targetTicks);

        drive.drive_robot(0, 0, 0, 0);
    }
}
