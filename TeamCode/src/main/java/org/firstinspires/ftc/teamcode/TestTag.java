package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetPower;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;
@Disabled
@Autonomous(name="BlueTeam", group = "Concept")
public class TestTag extends LinearOpMode {

    final double DESIRED_DISTANCE = 12.0; // inches
    final double SPEED_GAIN  = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN   = 0.01;

    final double MAX_AUTO_SPEED = 0.5;
    final double MAX_AUTO_STRAFE= 0.5;
    final double MAX_AUTO_TURN  = 0.3;

    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 585;   // ✅ Only look for tag ID 585

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    double left_motor = 0.0, right_motor = 0.0;
    FlyWheel_Launch_SetPower launch = new FlyWheel_Launch_SetPower();

    @Override
    public void runOpMode() {
        boolean targetFound = false;
        double drive = 0, strafe = 0, turn = 0;

        initAprilTag();

        frontLeftDrive = hardwareMap.get(DcMotor.class, "f_l_dr");
        frontRightDrive = hardwareMap.get(DcMotor.class, "f_r_dr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "b_l_dr");
        backRightDrive = hardwareMap.get(DcMotor.class, "b_r_dr");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        if (USE_WEBCAM)
            setManualExposure(6, 250);

        telemetry.addData(">", "Touch START to begin");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            targetFound = false;
            desiredTag = null;

            // Scan for AprilTags
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null && detection.id == DESIRED_TAG_ID) {
                    targetFound = true;
                    desiredTag = detection;
                    break;
                }
            }

            if (targetFound) {
                telemetry.addData("Tag Found", "ID %d", desiredTag.id);

                double rangeError   = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                double headingError = desiredTag.ftcPose.bearing;
                double yawError     = desiredTag.ftcPose.yaw;

                drive  = Range.clip(-rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(-headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);
                strafe = Range.clip(yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

// Stop small rotation/strafe jitter when tag is centered
                if (Math.abs(headingError) < 1.5) turn = 0;    // degrees threshold for alignment
                if (Math.abs(yawError) < 1.0) strafe = 0;

// Stop if close enough to tag
                if (Math.abs(rangeError) < 2.0) {
                    drive = 0;
                    strafe = 0;
                    turn = 0;
                    telemetry.addData("Status", "At target distance");
                    left_motor = 0.55;
                    right_motor = 0.55;
                    launch.setMotorSpeed(left_motor, right_motor);
                }


            } else {
                // Rotate to search for tag
                drive = 0;
                strafe = 0;
                turn = 0.3; // rotate in place slowly
                telemetry.addData("Searching", "Rotating to find tag ID %d...", DESIRED_TAG_ID);
            }

            telemetry.addData("Drive", "%.2f", drive);
            telemetry.addData("Strafe", "%.2f", strafe);
            telemetry.addData("Turn", "%.2f", turn);
            telemetry.update();

            moveRobot(drive, strafe, turn);

            sleep(10);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        double frontLeftPower  = x + y + yaw;
        double frontRightPower = x - y - yaw;
        double backLeftPower   = x - y + yaw;
        double backRightPower  = x + y - yaw;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        frontLeftDrive.setPower(frontLeftPower / max);
        frontRightDrive.setPower(frontRightPower / max);
        backLeftDrive.setPower(backLeftPower / max);
        backRightDrive.setPower(backRightPower / max);
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();
        aprilTag.setDecimation(2);

        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    private void setManualExposure(int exposureMS, int gain) {
        if (visionPortal == null) return;

        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            while (!isStopRequested() &&
                    (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
        }

        if (!isStopRequested()) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }
}
