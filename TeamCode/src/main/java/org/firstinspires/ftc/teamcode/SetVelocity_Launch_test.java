package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetVelocity_Debug;

@TeleOp(name = "SetVelocity Launch test", group = "TeleOp")
public class SetVelocity_Launch_test extends OpMode {

    FlyWheel_Launch_SetVelocity_Debug launch = new FlyWheel_Launch_SetVelocity_Debug();
    double targetRPM = 0;

    @Override
    public void init() {
        launch.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Choose base target RPM
        if (gamepad1.right_bumper) {
            targetRPM = 3000;
        } else if (gamepad1.right_trigger > 0.5) {
            targetRPM = 2000;
        } else if (gamepad1.left_bumper) {
            targetRPM = 1000;
        } else if (gamepad1.left_trigger > 0.5) {
            targetRPM = 0;
        }

        // Use the stabilizing version instead of raw setVelocity
        launch.stabilizeVelocity(targetRPM);

        telemetry.addData("Battery", launch.getBatteryVoltage());
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Motor1 Velocity", launch.getMotor1Velocity());
        telemetry.addData("Motor2 Velocity", launch.getMotor2Velocity());
        telemetry.update();
    }
}
