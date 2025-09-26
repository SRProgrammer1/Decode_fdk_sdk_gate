package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestBench;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench1;
@Disabled
@TeleOp
public class DcMotor extends OpMode {
    TestBench bench = new TestBench();
    TestBench1 bench1 = new TestBench1();
    @Override
    public void init() {
        bench.init(hardwareMap);
        bench1.init(hardwareMap);
    }

    @Override
    public void loop() {
        bench.setMotorSpeed(0.5);
        bench1.setMotorSpeed(0.5);
    }
}
