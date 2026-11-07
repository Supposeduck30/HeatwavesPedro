package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Find the flower servo positions without redeploying for every guess. Move the servo live,
 * save the REST and ACTIVE positions you like, and jump between them. Nothing is saved to the
 * robot: copy the final values into FLOWER_REST and FLOWER_ACTIVE in BioBuzzConfig.
 *
 * gamepad1 (Logitech names in brackets):
 *   dpad up / down         = position + / - one step
 *   dpad right / left      = bigger / smaller step (0.001, 0.005, 0.01, 0.05)
 *   left stick Y           = hold to sweep the servo slowly (push up = higher position)
 *   cross (A)              = save the current position as ACTIVE
 *   square (X)             = save the current position as REST
 *   circle (B)             = jump to the saved ACTIVE position
 *   triangle (Y)           = jump to the saved REST position
 *   right bumper           = go to 0.0 (hard limit check)
 *   left bumper            = go to 1.0 (hard limit check)
 *
 * HOW TO TUNE:
 *   1. Start with the servo free (nothing it can push into). Use the small steps to find the REST
 *      position, then press square (X) to save it.
 *   2. Move to the ACTIVE position the same way and press cross (A) to save it.
 *   3. Press circle / triangle to jump between them and check it looks right on the robot.
 *   4. Copy the "Paste into BioBuzzConfig" lines over FLOWER_REST and FLOWER_ACTIVE.
 *
 * If the servo buzzes or strains at an end, back off a little. Do not push it into a hard stop.
 */
@TeleOp(name = "Flower Servo Tuner", group = "BioBuzz")
public class FlowerServoTuner extends OpMode {

    private static final double[] STEPS = {0.001, 0.005, 0.01, 0.05};
    /** How fast the left stick sweeps the servo, position units per second at full push. */
    private static final double STICK_RATE_PER_SEC = 0.25;

    private Servo flower;

    private double position = FLOWER_REST;
    private double savedRest = FLOWER_REST;
    private double savedActive = FLOWER_ACTIVE;
    private int stepIndex = 2; // starts at step 0.01

    private long lastLoopNs;
    private boolean lastUp, lastDown, lastLeft, lastRight;
    private boolean lastCross, lastSquare, lastCircle, lastTriangle;
    private boolean lastLb, lastRb;

    @Override
    public void init() {
        flower = hardwareMap.get(Servo.class, FLOWER);
        flower.setPosition(position);
        lastLoopNs = System.nanoTime();

        telemetry.addLine("Flower Servo Tuner ready. Starting at the current FLOWER_REST.");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.nanoTime();
        double dt = (now - lastLoopNs) / 1e9;
        lastLoopNs = now;

        /* ---------- INPUT ---------- */
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean cross = gamepad1.cross, square = gamepad1.square;
        boolean circle = gamepad1.circle, triangle = gamepad1.triangle;
        boolean lb = gamepad1.left_bumper, rb = gamepad1.right_bumper;

        if (right && !lastRight) stepIndex = Math.min(stepIndex + 1, STEPS.length - 1);
        if (left && !lastLeft) stepIndex = Math.max(stepIndex - 1, 0);

        if (up && !lastUp) position += STEPS[stepIndex];
        if (down && !lastDown) position -= STEPS[stepIndex];

        // Slow sweep with the stick (deadzone so it does not drift)
        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) > 0.1) position += stick * STICK_RATE_PER_SEC * dt;

        if (cross && !lastCross) savedActive = round3(position);
        if (square && !lastSquare) savedRest = round3(position);
        if (circle && !lastCircle) position = savedActive;
        if (triangle && !lastTriangle) position = savedRest;
        if (rb && !lastRb) position = 0.0;
        if (lb && !lastLb) position = 1.0;

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        lastCross = cross; lastSquare = square; lastCircle = circle; lastTriangle = triangle;
        lastLb = lb; lastRb = rb;

        position = Range.clip(position, 0.0, 1.0);
        flower.setPosition(position);

        /* ---------- TELEMETRY ---------- */
        telemetry.addData(">>> FLOWER POSITION", "%.3f", position);
        telemetry.addData("Step", "%.3f   (dpad right = bigger, left = smaller)", STEPS[stepIndex]);
        telemetry.addLine();
        telemetry.addData("Saved REST   (square / X)", "%.3f   [triangle / Y to jump]", savedRest);
        telemetry.addData("Saved ACTIVE (cross / A)", "%.3f   [circle / B to jump]", savedActive);
        telemetry.addData("Config now", "REST %.3f / ACTIVE %.3f", FLOWER_REST, FLOWER_ACTIVE);
        telemetry.addLine();
        telemetry.addLine("Paste into BioBuzzConfig:");
        telemetry.addLine(String.format("  FLOWER_REST   = %.3f;", savedRest));
        telemetry.addLine(String.format("  FLOWER_ACTIVE = %.3f;", savedActive));
        telemetry.addLine();
        telemetry.addLine("dpad up/down = move   left stick = sweep   bumpers = 0.0 / 1.0");
        telemetry.update();
    }

    private static double round3(double v) {
        return Math.round(v * 1000.0) / 1000.0;
    }
}
