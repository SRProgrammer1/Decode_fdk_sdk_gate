package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive_Robot;

import java.util.List;

@TeleOp(name = "AprilTag Move To Target (MecanumDrive_Robot)", group = "Test")
@Disabled

public class AprilTagDetection_MoveToTarget extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private MecanumDrive_Robot robot = new MecanumDrive_Robot();

    // --- Tunable settings ---
    private static final double DESIRED_DISTANCE = 10.0;  // inches from tag
    private static final double DRIVE_GAIN = 0.03;
    private static final double STRAFE_GAIN = 0.03;
    private static final double TURN_GAIN = 0.015;
    private static final double MAX_SPEED = 0.4;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Setup vision
        aprilTag = new AprilTagProcessor.Builder().build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("🚦 Ready to start. Look at an AprilTag.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.isEmpty()) {
                robot.drive_robot(0, 0, 0, 0);
                telemetry.addLine("❌ No tag detected.");
                telemetry.update();
                continue;
            }

            // Take the first detected tag
            AprilTagDetection tag = detections.get(0);

            double range = tag.ftcPose.range;   // forward distance (in)
            double bearing = tag.ftcPose.bearing; // left/right angle (deg)
            double yaw = tag.ftcPose.yaw;       // tag rotation (deg)

            telemetry.addData("Tag ID", tag.id);
            telemetry.addData("Range (in)", "%.1f", range);
            telemetry.addData("Bearing (deg)", "%.1f", bearing);
            telemetry.addData("Yaw (deg)", "%.1f", yaw);

            if (gamepad1.a) {
                // Calculate movement based on AprilTag data
                double rangeError = range - DESIRED_DISTANCE;
                double drive = rangeError * DRIVE_GAIN;         // forward/back
                double strafe = -yaw * STRAFE_GAIN;             // left/right adjust
                double turn = -bearing * TURN_GAIN;             // rotate to face tag

                // Move robot
                robot.drive_robot(drive, strafe, turn, MAX_SPEED);
                telemetry.addData("Action", "Moving toward tag...");
            } else {
                robot.drive_robot(0, 0, 0, 0);
                telemetry.addData("Idle", "Press A to move toward tag");
            }

            telemetry.update();
            sleep(20);
        }
    }
}
