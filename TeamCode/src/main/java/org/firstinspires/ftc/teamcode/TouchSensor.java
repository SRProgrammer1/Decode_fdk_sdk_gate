package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.touchsensor;

@TeleOp
public class TouchSensor extends OpMode {
    touchsensor sensor = new touchsensor();

    @Override
    public void init() {
        sensor.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Touch Sensor State", sensor.getTouchSensorState());
    }
}
