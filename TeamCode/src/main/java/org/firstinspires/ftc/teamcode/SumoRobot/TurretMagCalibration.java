package org.firstinspires.ftc.teamcode.SumoRobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="Calibration: Find Red Mag Angle", group="Calibration")
public class TurretMagCalibration extends LinearOpMode {

    private DcMotor turretMotor;
    private DigitalChannel limitSwitch;

    // MATCHES YOUR NEW TURRET BOT MATH EXACTLY
    private static final double COUNTS_PER_DEGREE = 366.0 / 90.0;

    @Override
    public void runOpMode() {
        turretMotor = hardwareMap.get(DcMotor.class, "Turret");
        limitSwitch = hardwareMap.get(DigitalChannel.class, "turretLimit");

        // Match the exact motor setup of TurretControllerRED
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("=== INSTRUCTIONS ===");
        telemetry.addLine("1. Manually center turret pointing STRAIGHT FORWARD.");
        telemetry.addLine("2. Press Start button on Driver Station.");
        telemetry.addLine("3. Move stick slowly to find the magnet.");
        telemetry.update();

        waitForStart();

        double triggerAngle = 0;
        boolean foundTrigger = false;

        while (opModeIsActive()) {
            // Read raw sensor (False/Low = Magnet Detected on standard pull-up configurations)
            boolean isTriggered = !limitSwitch.getState();

            // Calculate current relative angle (0.0 is where you initialized it pointing forward)
// NEW LINE: Automatically treats your starting position as 90 degrees!
            double currentAngle = 90.0 + (turretMotor.getCurrentPosition() / COUNTS_PER_DEGREE);
            // Slow manual drive via left stick X
            turretMotor.setPower(-gamepad1.left_stick_x * 0.25);

            if (isTriggered && !foundTrigger) {
                // Leading edge caught
                triggerAngle = currentAngle;
                foundTrigger = true;
            } else if (!isTriggered) {
                foundTrigger = false;
            }

            telemetry.addLine("=== RED TURRET CALIBRATION ===");
            telemetry.addData("LIMIT SWITCH DETECTED", isTriggered ? "YES (MAGNET FOUND)" : "NO");
            telemetry.addData("Live Angle Position", "%.2f°", currentAngle);

            telemetry.addLine("\n-------------------------------------------");
            telemetry.addData("COPY THIS VALUE", "%.2f", triggerAngle);
            telemetry.addLine("-------------------------------------------");

            telemetry.addLine("\nPaste this number into 'HOMING_ANGLE_DEGREES'");
            telemetry.addLine("inside your TurretControllerRED class.");
            telemetry.update();
        }
    }
}