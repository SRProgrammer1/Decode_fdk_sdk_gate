/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Unused;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetVelocity;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: front_left_drive and front_right_drive, back_left_drive and back_right_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID.
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */

@Autonomous(name="BLUE_FAR_CAMERA_TargetReached2", group = "Concept")
@Disabled
public class BLUE_FAR_CAMERA_TargetReached2 extends LinearOpMode
{
    // Adjust these numbers to suit your robot.

    static final double TICKS_PER_REV = 537.7;
    static final double WHEEL_DIAMETER_INCHES = 3;
    static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
    static final double ROBOT_TRACK_WIDTH_INCHES = 15.0;

    // ----- CONFIGURABLE PARAMETERS -----
    // Tag and camera
    private static final double TAG_SIZE_M = 0.1524; // physical tag side, meters (6 in). Change if your tags are different.
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 480;

    // Target pose (user requested): Range = 121.5 cm, Yaw = 31 deg, Bearing = -3 deg
    private static final double TARGET_RANGE_M = 1.215;    // 121.5 cm -> 1.215 m
    private static final double TARGET_YAW_DEG  = 31.0;    // degrees
    private static final double TARGET_BEAR_DEG = -3.0;    // degrees

    // Control gains (P controllers) — tune on robot
    private static final double K_DRIVE  = 0.6;   // meters -> forward speed (scaled, tune carefully)
    private static final double K_STRAFE = 0.9;   // meters -> lateral speed
    private static final double K_TURN   = 0.015; // degrees -> rotation speed

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private static final int DESIRED_TAG_ID = 20;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    // Tolerances for marking target reached
    private static final double RANGE_TOL_M = 0.05;   // 5 cm tolerance
    private static final double YAW_TOL_DEG = 2.0;    // degrees tolerance
    private static final double BEAR_TOL_DEG = 2.0;   // degrees tolerance

    private static final double MAX_FORWARD = 0.4;
    private static final double MAX_STRAFE  = 0.4;
    private static final double MAX_TURN    = 0.4;


    // flag set when we reach desired pose
    private boolean targetReached = false;

    ServoBench kicker = new ServoBench();
    MecanumDrive_Robot drive = new MecanumDrive_Robot();
    double targetRPM = 3000;   // Launch power

    FlyWheel_Launch_SetVelocity flywheel = new FlyWheel_Launch_SetVelocity();
    Ramp_Servo servo = new Ramp_Servo();
    intake_dcmotor intake = new intake_dcmotor();
    boolean lastButtonState = false;
    boolean lastButtonState2 = false;
    double forward, right, rotate;
    double maxSpeed = 0.5;
    double start_stop = 0.0;
    double left_motor = 0.0, right_motor = 0.0;

    boolean on_off = false;
    boolean on_off2 = false;

    boolean stop_drive = false;
    boolean first_launch = false;



    @Override public void runOpMode() {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected

        drive.init(hardwareMap);
        flywheel.init(hardwareMap);
        servo.init(hardwareMap);
        intake.init(hardwareMap);
        kicker.init(hardwareMap);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the Apriltag Detection process
        initAprilTag();


        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.update();
        waitForStart();
        //driveDistance(-68, 0.3);
        //sleep(1500);
        flywheel.setVelocityRPM(targetRPM);

        while (opModeIsActive()) {


            boolean currentButtonState = gamepad2.a;
            boolean currentButtonState2 = gamepad2.x;


            forward = gamepad1.left_stick_y;
            right = gamepad1.right_stick_x;
            rotate = gamepad1.left_stick_x;


            targetFound = false;
            desiredTag = null;


            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>", "HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range", "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing", "%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw", "%3.0f degrees", desiredTag.ftcPose.yaw);
            } else {

                telemetry.addData("Searching", "Rotating to find tag...");

            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
               // AprilTagDetectionPipeline.TagData tag = tagPipeline.getLatestDetection();


                // NOTE: Many SDKs represent ftcPose in meters. If your SDK uses inches, convert accordingly.
                double range_m = desiredTag.ftcPose.range;      // meters (expected)
                double yaw_deg = desiredTag.ftcPose.yaw;        // degrees
                double bearing_deg = desiredTag.ftcPose.bearing;// degrees

                // compute errors (we want pose -> target)
                double errRange = range_m - TARGET_RANGE_M;         // positive => we are farther than target
                double errYaw = yaw_deg - TARGET_YAW_DEG;        // degrees
                double errBear = bearing_deg - TARGET_BEAR_DEG;   // degrees

                // control (P controllers)
                double forward = Range.clip(errRange * K_DRIVE, -MAX_FORWARD, MAX_FORWARD);    // positive moves forward
                double strafe = Range.clip(-/* sign to correct axis */ (errYaw * (Math.PI / 180.0)) * K_STRAFE, -MAX_STRAFE, MAX_STRAFE);
                // Note: we used yaw → lateral correction. Alternately use chosen.ftcPose.x for direct lateral offset if desired:
                // double strafeCmd = Range.clip(-chosen.ftcPose.x * K_STRAFE, -MAX_STRAFE, MAX_STRAFE);
                double turn = Range.clip(-errBear * K_TURN, -MAX_TURN, MAX_TURN); // negative because positive bearing usually means tag to left (adjust after testing)

                // telemetry for tuning
                telemetry.addData("TagID", desiredTag.id);
                telemetry.addData("Range_m", "%.3f / %.3f", range_m, TARGET_RANGE_M);
                telemetry.addData("Yaw_deg", "%.2f / %.2f", yaw_deg, TARGET_YAW_DEG);
                telemetry.addData("Bearing_deg", "%.2f / %.2f", bearing_deg, TARGET_BEAR_DEG);
                telemetry.addData("ErrRange_m", "%.3f", errRange);
                telemetry.addData("ErrYaw_deg", "%.2f", errYaw);
                telemetry.addData("ErrBear_deg", "%.2f", errBear);
                telemetry.addData("Cmds fwd/strafe/turn", "%.3f / %.3f / %.3f", forward, strafe, turn);
                telemetry.update();

                // check reached condition
                boolean withinRange = Math.abs(errRange) <= RANGE_TOL_M;
                boolean withinYaw = Math.abs(errYaw) <= YAW_TOL_DEG;
                boolean withinBear = Math.abs(errBear) <= BEAR_TOL_DEG;

                if (withinRange && withinYaw && withinBear) {
                    forward = 0;
                    strafe = 0;
                    turn = 0;
                    stop_drive = true;
                    telemetry.addData("Status", "At target distance");
                }

                sleep(6000);

                if (stop_drive && (!first_launch)) {

                    // First Launch
                    intake.setMotorSpeed_intake(1.0);
                    kicker.setServoRot(1.0);
                    servo.setServo_ramp(1.0);
                    sleep(6000);
                    first_launch = true;

                }

            }
        }
    }


            /**
             * Initialize the AprilTag processor.
             */
            private void initAprilTag () {
                // Create the AprilTag processor by using a builder.
                aprilTag = new AprilTagProcessor.Builder().build();

                // Adjust Image Decimation to trade-off detection-range for detection-rate.
                // e.g. Some typical detection data using a Logitech C920 WebCam
                // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
                // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
                // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
                // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
                // Note: Decimation can be changed on-the-fly to adapt during a match.
                aprilTag.setDecimation(2);

                // Create the vision portal by using a builder.
                if (USE_WEBCAM) {
                    visionPortal = new VisionPortal.Builder()
                            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                            .setCameraResolution(new Size(640, 480))
                            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                            .addProcessor(aprilTag)
                            .build();
                } else {
                    visionPortal = new VisionPortal.Builder()
                            .setCamera(BuiltinCameraDirection.BACK)
                            .addProcessor(aprilTag)
                            .build();
                }
            }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
            private void setManualExposure ( int exposureMS, int gain){
                // Wait for the camera to be open, then use the controls

                if (visionPortal == null) {
                    return;
                }

                // Make sure camera is streaming before we try to set the exposure controls
                if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    telemetry.addData("Camera", "Waiting");
                    telemetry.update();
                    while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                        sleep(20);
                    }
                    telemetry.addData("Camera", "Ready");
                    telemetry.update();
                }

                // Set camera controls unless we are stopping.
                if (!isStopRequested()) {
                    ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                    if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                        exposureControl.setMode(ExposureControl.Mode.Manual);
                        sleep(50);
                    }
                    exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
                    sleep(20);
                    GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                    gainControl.setGain(gain);
                    sleep(20);
                }
            }
            private void driveDistance ( double inches, double speed){
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

            private void turnDegreesLeft ( double degrees, double speed){
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

            private void turnDegreesRight ( double degrees, double speed){
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

        }