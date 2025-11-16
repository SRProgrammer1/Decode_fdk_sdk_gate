package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.FlyWheel_Launch_SetVelocity_test;
import org.firstinspires.ftc.teamcode.mechanisms.Ramp_Servo;
import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.intake_dcmotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;


@TeleOp(name = "SetVelocity Launch Battery1", group = "TeleOp")
public class SetVelocity_Launch_Battery1 extends OpMode {
    FlyWheel_Launch_SetVelocity_test launch = new FlyWheel_Launch_SetVelocity_test();


    ServoBench kicker = new ServoBench();
    Ramp_Servo servo = new Ramp_Servo();
    intake_dcmotor intake = new intake_dcmotor();
    private VoltageSensor batteryVoltageSensor;


    double targetRPM = 0;
    double voltage = 0;

    double count;
    boolean yWasPressed = false;
    boolean bWasPressed = false;


    @Override
    public void init() {
        launch.init(hardwareMap);
        servo.init(hardwareMap);
        intake.init(hardwareMap);
        kicker.init(hardwareMap);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

   /* private double rpmForHighVoltage(double normalRPM) {
        if (voltage > 13.0) {
            // Map normal RPM to reduced high-voltage RPM
            if (normalRPM == 1200) return 960;
            if (normalRPM == 1100) return 900;
            if (normalRPM == 900) return 780;
        } else if (voltage > 12.0){
            if (normalRPM == 1200) return 1000;
            if (normalRPM == 1100) return 900;
            if (normalRPM == 900) return 800;
        }

        return normalRPM;
    }*/

    @Override
    public void loop() {

        voltage = batteryVoltageSensor.getVoltage();

        // === Step 5: Stop all mechanisms ===
        //flywheel.setMotorSpeed(0.0, 0.0);
// === One-press RPM control ===

// A = +20 RPM once
        if (gamepad1.y && !yWasPressed) {
            targetRPM += 10;
        }
        yWasPressed = gamepad1.y;

// B = -20 RPM once
        if (gamepad1.b && !bWasPressed) {
            targetRPM -= 10;
        }
        bWasPressed = gamepad1.b;

        if (gamepad1.right_bumper) {
            intake.setMotorSpeed_intake(1.0);
            kicker.setServoRot(0.15);
            servo.setServo_ramp(1.0);

            targetRPM = 2150;

        } else if (gamepad1.right_trigger > 0.5) {
            intake.setMotorSpeed_intake(1.0);
            kicker.setServoRot(0.15);
            servo.setServo_ramp(1.0);

            targetRPM = 2000;

        } else if (gamepad1.left_bumper) {
            intake.setMotorSpeed_intake(1.0);
            kicker.setServoRot(0.15);
            servo.setServo_ramp(1.0);

            targetRPM = 1900;

        } else if (gamepad1.left_trigger > 0.5) {
            targetRPM = 0;
            intake.setMotorSpeed_intake(0);
            kicker.setServoRot(0.0);
            servo.setServo_ramp(0.0);
        }


        final double normalized_voltage = 13.6;

        final double voltageNormalizedCoefficient = normalized_voltage / voltage;

        // Apply target RPM (motor2 will spin opposite automatically)

        launch.setVelocityRPM(targetRPM * voltageNormalizedCoefficient);

        telemetry.addData("Battery Voltage", "%.2f V", voltage);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Motor1 Velocity", launch.getMotor1Velocity() / 28.0 * 60.0);
        telemetry.addData("Motor1 Velocity TPS", launch.getMotor1Velocity());
        telemetry.addData("Motor2 Velocity", launch.getMotor2Velocity() / 28.0 * 60.0);
        telemetry.update();
    }
}
//@Disabled
