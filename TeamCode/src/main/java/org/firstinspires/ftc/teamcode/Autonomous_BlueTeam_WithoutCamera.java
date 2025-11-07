package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetPower;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;

@Autonomous(name = "Autonomous_BLUE_Drive_WithoutCAM", group = "Auto")
public class Autonomous_BlueTeam_WithoutCamera extends LinearOpMode {

    private MecanumDrive_Robot drive = new MecanumDrive_Robot();
    private Ramp_Servo servo = new Ramp_Servo();
    private FlyWheel_Launch_SetPower flywheel = new FlyWheel_Launch_SetPower();
    private intake_dcmotor intake = new intake_dcmotor();
    private ServoBench kicker = new ServoBench();

    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3.78;
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
            driveDistance(-48, 0.3);
            flywheel.setMotorSpeed(0.40, 0.40);




            // === Step 3: Start flywheels ===
            telemetry.addLine("Starting flywheels...");
            telemetry.update();

            sleep(4000);

            // === Step 4: Start intake + kicker + ramp ===
            telemetry.addLine("Starting intake and kicker...");
            telemetry.update();
            intake.setMotorSpeed_intake(1.0);
            kicker.setServoRot(1.0);
            servo.setServo_ramp(1.0);
            sleep(5000);

            // === Step 5: Stop all mechanisms ===
            flywheel.setMotorSpeed(0.0, 0.0);
            intake.setMotorSpeed_intake(0);
            kicker.setServoRot(0.0);
            servo.setServo_ramp(0.0);

            telemetry.addLine("Shooting complete. Moving backward...");
            telemetry.update();
            sleep(500);

            turnDegreesLeft(300, 0.4);
            sleep(500);

            // === Step 6: Drive backward 24 inches (was forward) ===
            intake.setMotorSpeed_intake(1.0);
            servo.setServo_ramp(1.0);
            driveDistance(-34, 0.4);
            sleep(500);
            intake.setMotorSpeed_intake(0.0);
            servo.setServo_ramp(0.0);
            flywheel.setMotorSpeed(0.40, 0.40);
            driveDistance(34, 0.5);
            turnDegreesLeft(200, 0.3);
            sleep(500);
            servo.setServo_ramp(1.0);
            intake.setMotorSpeed_intake(1.0);
            kicker.setServoRot(1.0);
            sleep(5000);
            servo.setServo_ramp(0.0);
            intake.setMotorSpeed_intake(0.0);
            kicker.setServoRot(0.0);
            turnDegreesLeft(50, 0.5);
            driveDistance(20, 0.5);
            /* turnDegreesLeft(400, 0.5);
            intake.setMotorSpeed_intake(1.0);
            servo.setServo_ramp(1.0);
            driveDistance(-33, 0.5);
            sleep(500);
            driveDistance(33, 0.5);
            turnDegreesLeft(300, 0.5);



            telemetry.addLine("Autonomous routine complete!");
            telemetry.update();

             */
        }
    }

    // --- Helper methods using MecanumDrive_Robot ---

    private void driveDistance(double inches, double speed) {
        int targetTicks = (int) (Math.abs(inches) * TICKS_PER_INCH);

        // Reset encoders
        drive.getFrontLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getFrontRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.getFrontLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getFrontRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

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
        drive.getFrontLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getFrontRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.getFrontLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getFrontRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

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
        drive.getFrontLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getFrontRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.getBackRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        drive.getFrontLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getFrontRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackLeft().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);
        drive.getBackRight().setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER);

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
}
