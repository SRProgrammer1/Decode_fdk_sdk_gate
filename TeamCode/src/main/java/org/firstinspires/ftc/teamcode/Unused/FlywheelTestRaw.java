package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Flywheel Raw Speed Test", group = "Test")
@Disabled
public class FlywheelTestRaw extends OpMode {

    DcMotorEx m1, m2;

    @Override
    public void init() {
        m1 = hardwareMap.get(DcMotorEx.class, "fly_wheel_1");
        m2 = hardwareMap.get(DcMotorEx.class, "fly_wheel_2");

        m1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        m1.setDirection(DcMotorEx.Direction.FORWARD);
        m2.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void loop() {

        double batt = hardwareMap.voltageSensor.iterator().next().getVoltage();

        double power = 0;
        if (gamepad1.a) power = 1.0;     // full speed
        if (gamepad1.b) power = 0.5;     // half speed
        if (gamepad1.x) power = 0.25;    // quarter speed
        if (gamepad1.y) power = 0;       // stop

        m1.setPower(power);
        m2.setPower(power);

        telemetry.addData("Power", power);
        telemetry.addData("Battery Voltage", batt);

        telemetry.addData("M1 Velocity", m1.getVelocity());
        telemetry.addData("M2 Velocity", m2.getVelocity());

        telemetry.update();
    }
}
