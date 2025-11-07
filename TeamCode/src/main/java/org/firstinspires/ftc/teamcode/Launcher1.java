package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//@Disabled

@TeleOp(name="NeveRest 6000 Velocity Control", group="Examples")
public class Launcher1 extends LinearOpMode {

    DcMotorEx motor;
    static final int TICKS_PER_REV = 28;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "fly_wheel_1");

        // Reset and set mode
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Tune the built-in velocity PIDF
        motor.setVelocityPIDFCoefficients(5.0, 0.5, 0, 12.0);

        waitForStart();

        while (opModeIsActive()) {
            double targetRPM = 0;

            if (gamepad1.x) targetRPM = 180;
            else if (gamepad1.y) targetRPM = 240;
            else if (gamepad1.b) targetRPM = 600;
            else if (gamepad1.a) targetRPM = 0;

            // Convert RPM → ticks per second
            double ticksPerSecond = (targetRPM / 60.0) * TICKS_PER_REV;

            // Set motor velocity in ticks/sec
            motor.setVelocity(ticksPerSecond);

            // Telemetry
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Measured Ticks/sec", motor.getVelocity());
            telemetry.addData("Measured RPM", motor.getVelocity() * 60 / TICKS_PER_REV);
            telemetry.update();
        }
    }
}
