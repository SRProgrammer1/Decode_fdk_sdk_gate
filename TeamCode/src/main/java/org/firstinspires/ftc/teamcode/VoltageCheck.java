package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
public class VoltageCheck extends OpMode {

    private VoltageSensor batteryVoltageSensor;

    @Override
    public void init() {
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void loop() {
        double voltage = batteryVoltageSensor.getVoltage();
        telemetry.addData("Battery Voltage", "%.2f V", voltage);
        telemetry.update();
    }
}

