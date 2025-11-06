package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetVelocity;
@Disabled
@TeleOp(name = "SetVelocity Launch", group = "TeleOp")
public class SetVelocity_Launch extends OpMode {

    FlyWheel_Launch_SetVelocity launch = new FlyWheel_Launch_SetVelocity();
    double targetRPM = 0;

    @Override
    public void init() {
        launch.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.right_bumper) {
            targetRPM = 5000;
        } else if (gamepad1.right_trigger > 0.5) {
            targetRPM = 3000;
        } else if (gamepad1.left_bumper) {
            targetRPM = 1000;
        } else if (gamepad1.left_trigger > 0.5) {
            targetRPM = 0;
        }

        // Apply target RPM (motor2 will spin opposite automatically)
        launch.setMotorRPM(targetRPM);

        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Motor1 Velocity", launch.getMotor1Velocity());
        telemetry.addData("Motor2 Velocity", launch.getMotor2Velocity());
        telemetry.update();
    }
}
