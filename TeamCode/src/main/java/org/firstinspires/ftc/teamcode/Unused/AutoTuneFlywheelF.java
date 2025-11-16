// AutoTuneFlywheelF.java
package org.firstinspires.ftc.teamcode.Unused;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name="AutoTune Flywheel F", group="tuning")
@Disabled
public class AutoTuneFlywheelF extends LinearOpMode {

    private DcMotorEx m1, m2;

    @Override
    public void runOpMode() throws InterruptedException {
        m1 = hardwareMap.get(DcMotorEx.class, "fly_wheel_1");
        m2 = hardwareMap.get(DcMotorEx.class, "fly_wheel_2");

        m1.setDirection(DcMotorEx.Direction.FORWARD);
        m2.setDirection(DcMotorEx.Direction.REVERSE);

        m1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready. Press start to run motors at full power and measure velocity.");
        telemetry.update();
        waitForStart();

        // Spin both motors at full power, wait for them to settle, then sample
        m1.setPower(1.0);
        m2.setPower(1.0);

        sleep(1200); // let them spin up (adjust if needed)

        // sample for 1.5s
        long sampleStart = System.currentTimeMillis();
        long sampleDuration = 1500;
        double sum1 = 0, sum2 = 0;
        int count = 0;

        while (opModeIsActive() && (System.currentTimeMillis() - sampleStart) < sampleDuration) {
            double v1 = Math.abs(m1.getVelocity()); // ticks/sec
            double v2 = Math.abs(m2.getVelocity());
            sum1 += v1; sum2 += v2; count++;
            telemetry.addData("sample", "%d v1=%.1f v2=%.1f", count, v1, v2);
            telemetry.update();
            sleep(50); // sample every 50ms
        }

        // stop motors
        m1.setPower(0.0);
        m2.setPower(0.0);

        if (count == 0) {
            telemetry.addLine("No samples collected!");
            telemetry.update();
            return;
        }

        double avgV1 = sum1 / count;
        double avgV2 = sum2 / count;
        double avgV = (avgV1 + avgV2) / 2.0;

        telemetry.addData("Measured avg v1", "%.2f ticks/s", avgV1);
        telemetry.addData("Measured avg v2", "%.2f ticks/s", avgV2);
        telemetry.addData("Measured avg both", "%.2f ticks/s", avgV);

        // compute kF based on 32767 internal scale
        double kF = 32767.0 / Math.max(avgV, 1.0);

        // starting P & D recommended (you can change these)
        double P = 6.0;
        double I = 0.0;
        double D = 1.0;

        PIDFCoefficients pidf = new PIDFCoefficients(P, I, D, kF);

        // Apply to motors
        m1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        m2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);

        telemetry.addData("Computed kF", "%.3f", kF);
        telemetry.addData("Applied PIDF", "P=%.2f I=%.2f D=%.2f F=%.3f", P, I, D, kF);
        telemetry.update();

        // Keep telemetry visible until stop
        while (opModeIsActive()) {
            sleep(200);
        }
    }
}
