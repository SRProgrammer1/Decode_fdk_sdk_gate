package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetPower;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;
@Autonomous(name = "Autonomous1", group = "Auto")
@Disabled

public class AutonamousTest extends OpMode {
    ServoBench kicker = new ServoBench();
    MecanumDrive_Robot drive_r = new MecanumDrive_Robot();
    FlyWheel_Launch_SetPower launch = new FlyWheel_Launch_SetPower();
    Ramp_Servo servo = new Ramp_Servo();
    intake_dcmotor intake_motor = new intake_dcmotor();

    double startTime = 0.0;
    boolean driveStarted = false;
    boolean driveDone = false;

    // Encoder constants
    static final double COUNTS_PER_MOTOR_REV = 537.7;  // GoBilda 5202/5203 motors
    static final double WHEEL_DIAMETER_INCHES = 4.0;   // Wheel diameter
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_DISTANCE_INCHES = 48.0;  // 4 feet

    DcMotor frontLeft, frontRight, backLeft, backRight;

    @Override
    public void init() {
        drive_r.init(hardwareMap);
        launch.init(hardwareMap);
        servo.init(hardwareMap);
        intake_motor.init(hardwareMap);
        kicker.init(hardwareMap);

        // Access motors safely through getters
        frontLeft = drive_r.getFrontLeft();
        frontRight = drive_r.getFrontRight();
        backLeft = drive_r.getBackLeft();
        backRight = drive_r.getBackRight();

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        startTime = getRuntime();
    }

    @Override
    public void loop() {
        double elapsedTime = getRuntime() - startTime;

        // --- Phase 1: Flywheels ON (0–15 sec) ---
        if (elapsedTime <= 15.0) {
            launch.setMotorSpeed(0.48, 0.48);
        } else {
            launch.setMotorSpeed(0.0, 0.0);
        }

        // --- Phase 2: Intake + Ramp ON (5–15 sec) ---
        if (elapsedTime >= 5.0 && elapsedTime <= 15.0) {
            intake_motor.setMotorSpeed_intake(1.0);
            servo.setServo_ramp(1.0);
            kicker.setServoRot(1.0);
        } else {
            intake_motor.setMotorSpeed_intake(0.0);
            servo.setServo_ramp(0.0);
            kicker.setServoRot(0.0);
        }

        // --- Phase 3: Encoder Drive After 15 sec ---
        if (elapsedTime > 15.0 && !driveStarted && !driveDone) {
            driveStarted = true;

            int targetCounts = (int) (DRIVE_DISTANCE_INCHES * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetCounts);
            frontRight.setTargetPosition(frontRight.getCurrentPosition() + targetCounts);
            backLeft.setTargetPosition(backLeft.getCurrentPosition() + targetCounts);
            backRight.setTargetPosition(backRight.getCurrentPosition() + targetCounts);

            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontLeft.setPower(0.5);
            frontRight.setPower(0.5);
            backLeft.setPower(0.5);
            backRight.setPower(0.5);
        }

        // Stop once target is reached
        if (driveStarted && !driveDone &&
                !frontLeft.isBusy() && !frontRight.isBusy() &&
                !backLeft.isBusy() && !backRight.isBusy()) {

            frontLeft.setPower(0.0);
            frontRight.setPower(0.0);
            backLeft.setPower(0.0);
            backRight.setPower(0.0);

            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            driveDone = true;
        }

        // --- Telemetry ---
        telemetry.addData("Elapsed Time", elapsedTime);
        telemetry.addData("Flywheels", elapsedTime <= 15.0 ? "ON" : "OFF");
        telemetry.addData("Intake", (elapsedTime >= 5.0 && elapsedTime <= 15.0) ? "ON" : "OFF");
        telemetry.addData("Drive Started", driveStarted);
        telemetry.addData("Drive Done", driveDone);
        telemetry.addData("FL Target", frontLeft.getTargetPosition());
        telemetry.addData("FL Current", frontLeft.getCurrentPosition());
        telemetry.update();
    }
}
