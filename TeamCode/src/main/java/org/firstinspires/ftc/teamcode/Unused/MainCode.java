package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.ServoBench;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench;
import org.firstinspires.ftc.teamcode.mechanisms.TestBench1;
@Disabled

@TeleOp(name="Main Code (Velocity Control)", group="Linear OpMode")
public class MainCode extends LinearOpMode {

    ServoBench bench = new ServoBench();

    DcMotorEx motor;
    DcMotorEx motor1;
    DcMotor intakeMotor;

    TestBench bench1 = new TestBench();
    TestBench1 bench2 = new TestBench1();

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    @Override

    public void runOpMode() {

        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "f_l_dr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "b_l_dr");
        frontRightDrive = hardwareMap.get(DcMotor.class, "f_r_dr");
        backRightDrive = hardwareMap.get(DcMotor.class, "b_r_dr");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        bench.init(hardwareMap);
        bench1.init(hardwareMap);
        bench2.init(hardwareMap);

        motor = hardwareMap.get(DcMotorEx.class, "fly_wheel_1");
        motor1 = hardwareMap.get(DcMotorEx.class, "fly_wheel_2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Ticks per revolution for NeveRest or goBILDA motors (adjust if different)
        double TICKS_PER_REV = 28.0;
        double MAX_RPM = 6000.0; // example motor max speed
        double TICKS_PER_SEC_AT_MAX_RPM = (MAX_RPM / 60.0) * TICKS_PER_REV;

        while (opModeIsActive()) {
            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.right_stick_x;
            double yaw = gamepad1.left_stick_x;

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower),
                    Math.max(Math.abs(frontRightPower),
                            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            // Drive motors still use open-loop power
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            // --- FLYWHEEL VELOCITY CONTROL ---

            // example preset speeds in RPM
            double shootHighRPM = 5000;
            double shootMidRPM = 420;
            double shootLowRPM = 38;

            if (gamepad1.right_bumper) {
                motor.setVelocity((shootHighRPM / 60.0) * TICKS_PER_REV);
                motor1.setVelocity((shootHighRPM / 60.0) * TICKS_PER_REV);
            } else if (gamepad1.right_trigger > 0.5) {
                motor.setVelocity((shootMidRPM / 60.0) * TICKS_PER_REV);
                motor1.setVelocity((shootMidRPM / 60.0) * TICKS_PER_REV);
            } else if (gamepad1.left_trigger > 0.5) {
                motor.setVelocity((shootLowRPM / 60.0) * TICKS_PER_REV);
                motor1.setVelocity((shootLowRPM / 60.0) * TICKS_PER_REV);
            } else if (gamepad1.left_bumper) {
                motor.setVelocity(0);
                motor1.setVelocity(0);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Velocity (ticks/s)", motor.getVelocity());
            telemetry.addData("Target RPM", "%.1f", (motor.getVelocity() / TICKS_PER_REV) * 60.0);
            telemetry.update();
        }
    }
}
