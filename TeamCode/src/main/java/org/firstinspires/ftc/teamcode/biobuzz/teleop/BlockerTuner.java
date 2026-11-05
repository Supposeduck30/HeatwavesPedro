package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Find the blocker servo positions without redeploying for every guess.
 *
 * gamepad1:
 *   dpad up / down        = +/- 0.01
 *   right / left bumper   = +/- 0.05
 *   square                = remember the CURRENT position as "BLOCK"
 *   circle                = remember the CURRENT position as "OPEN"
 *   cross                 = jump to the remembered BLOCK position
 *   triangle (hold)       = go to the remembered OPEN position, back to BLOCK on release
 *
 * Telemetry shows the remembered values. Copy them into BLOCKER_BLOCK / BLOCKER_OPEN in
 * BioBuzzConfig (and KICKER_BLOCK / KICKER_OPEN in ScrimmageTeleop).
 * Keep the servo off any hard stop: if it buzzes or strains, back off a few hundredths.
 */
@TeleOp(name = "Blocker Tuner", group = "BioBuzz")
public class BlockerTuner extends OpMode {

    private Servo blocker;

    // Where the servo sits from init, so it doesn't slam to the config BLOCK position.
    private static final double START_POSITION = 0.95;

    private double position = START_POSITION;
    private double blockPos = BLOCKER_BLOCK;
    private double openPos = BLOCKER_OPEN;

    private boolean lastUp, lastDown, lastRb, lastLb, lastSquare, lastCircle, lastCross;

    @Override
    public void init() {
        blocker = hardwareMap.get(Servo.class, BLOCKER);
        blocker.setPosition(position);
        telemetry.addLine("Blocker Tuner ready.");
        telemetry.addData("Init position", "%.3f", START_POSITION);
        telemetry.update();
    }

    @Override
    public void start() {
        blocker.setPosition(position);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up && !lastUp) position += 0.01;
        if (gamepad1.dpad_down && !lastDown) position -= 0.01;
        if (gamepad1.right_bumper && !lastRb) position += 0.05;
        if (gamepad1.left_bumper && !lastLb) position -= 0.05;
        if (gamepad1.square && !lastSquare) blockPos = position;
        if (gamepad1.circle && !lastCircle) openPos = position;
        if (gamepad1.cross && !lastCross) position = blockPos;

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastRb = gamepad1.right_bumper;
        lastLb = gamepad1.left_bumper;
        lastSquare = gamepad1.square;
        lastCircle = gamepad1.circle;
        lastCross = gamepad1.cross;

        position = Range.clip(position, 0.0, 1.0);

        // Holding triangle previews the open position without losing the position being tuned.
        double commanded = gamepad1.triangle ? openPos : position;
        blocker.setPosition(commanded);

        telemetry.addData("Commanded", "%.3f", commanded);
        telemetry.addData("Tuning position", "%.3f", position);
        telemetry.addData("Remembered BLOCK (square)", "%.3f", blockPos);
        telemetry.addData("Remembered OPEN (circle)", "%.3f", openPos);
        telemetry.addData("In config now", "BLOCK %.3f / OPEN %.3f", BLOCKER_BLOCK, BLOCKER_OPEN);
        telemetry.update();
    }
}
