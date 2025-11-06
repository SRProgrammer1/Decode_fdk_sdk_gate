package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "Drive To AprilTag 19 (Full Align + 40 Inches)", group = "Auto")
@Disabled

public class AutoTest extends LinearOpMode {

    private AprilTagWebcam aprilTagWebcam = new AprilTagWebcam();
    private MecanumDrive_Robot drive = new MecanumDrive_Robot();

    private static final int TARGET_TAG_ID = 19;          // <-- Tag ID changed to 19
    private static final double TARGET_DISTANCE_INCHES = 40.0;
    private static final double MAX_SPEED = 0.4;

    private static final double DIST_TOLERANCE = 1.0;   // inches
    private static final double ANGLE_TOLERANCE = 2.0;  // degrees
    private static final double STRAFE_TOLERANCE = 1.0; // inches (side-to-side offset)

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTagWebcam.init(hardwareMap, telemetry);
        drive.init(hardwareMap);

        telemetry.addLine("Initializing AprilTag camera...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            aprilTagWebcam.update();
            AprilTagDetection tag = aprilTagWebcam.getTagBySpecificId(TARGET_TAG_ID);

            if (tag != null && tag.ftcPose != null) {
                double distance = tag.ftcPose.range; // inches
                double yaw = tag.ftcPose.yaw;        // degrees (positive = rotated left)
                double xOffset = tag.ftcPose.x;      // side-to-side offset (inches)

                double distanceError = distance - TARGET_DISTANCE_INCHES;
                double angleError = yaw;
                double strafeError = -xOffset; // negative so right is positive

                telemetry.addData("Tag ID", tag.id);
                telemetry.addData("Distance (in)", distance);
                telemetry.addData("Yaw (deg)", yaw);
                telemetry.addData("X Offset (in)", xOffset);
                telemetry.addData("Errors", "Dist: %.2f, Yaw: %.2f, Strafe: %.2f",
                        distanceError, angleError, strafeError);
                telemetry.update();

                // === STEP 1: ROTATE TO FACE TAG ===
                if (Math.abs(angleError) > ANGLE_TOLERANCE) {
                    double turnPower = angleError * 0.02;
                    turnPower = Math.max(-0.25, Math.min(0.25, turnPower));
                    drive.drive_robot(0, 0, -turnPower, 1.0);
                    continue;
                }

                // === STEP 2: STRAFE TO CENTER TAG ===
                if (Math.abs(strafeError) > STRAFE_TOLERANCE) {
                    double strafePower = strafeError * 0.02;
                    strafePower = Math.max(-0.3, Math.min(0.3, strafePower));
                    drive.drive_robot(0, strafePower, 0, 1.0);
                    continue;
                }

                // === STEP 3: DRIVE TO TARGET DISTANCE ===
                if (Math.abs(distanceError) > DIST_TOLERANCE) {
                    double movePower = distanceError * 0.02;
                    movePower = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, movePower));
                    drive.drive_robot(movePower, 0, 0, 1.0);
                } else {
                    drive.drive_robot(0, 0, 0, 0);
                    telemetry.addLine("✅ Perfectly aligned to Tag 19 and 40 inches away!");
                    telemetry.update();
                    break;
                }

            } else {
                drive.drive_robot(0, 0, 0, 0);
                telemetry.addLine("Tag 19 not visible — waiting...");
                telemetry.update();
            }

            sleep(50);
        }

        drive.drive_robot(0, 0, 0, 0);
        aprilTagWebcam.stop();
    }
}
