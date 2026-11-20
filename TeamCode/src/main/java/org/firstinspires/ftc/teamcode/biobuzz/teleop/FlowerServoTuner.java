package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Find the two flower servo positions without redeploying for every guess. Move a servo live,
 * save its REST and ACTIVE positions, and jump between them. Nothing is saved to the robot:
 * copy the final values into BioBuzzConfig.
 *
 * gamepad1:
 *   options (or share)     = switch which servo you are tuning (Flower1 / Flower2)
 *   touchpad               = hold the selected servo's saved positions on BOTH servos (see below)
 *   dpad up / down         = position + / - one step
 *   dpad right / left      = bigger / smaller step (0.001, 0.005, 0.01, 0.05)
 *   left stick Y           = hold to sweep the servo slowly (push up = higher position)
 *   cross (A)              = save the current position as ACTIVE
 *   square (X)             = save the current position as REST
 *   circle (B)             = jump the selected servo to its saved ACTIVE position
 *   triangle (Y)           = jump the selected servo to its saved REST position
 *   right bumper           = selected servo to 0.0 (hard limit check)
 *   left bumper            = selected servo to 1.0 (hard limit check)
 *   right trigger          = BOTH servos to their saved ACTIVE positions (check they match up)
 *   left trigger           = BOTH servos to their saved REST positions
 *
 * HOW TO TUNE:
 *   1. Start with the servos free (nothing they can push into). Pick Flower1, use small steps to
 *      find REST, press square (X), find ACTIVE, press cross (A).
 *   2. Press options to switch to Flower2 and repeat.
 *   3. Use the triggers to move both together and check they line up on the robot.
 *   4. Copy the "Paste into BioBuzzConfig" lines over the FLOWER_* / FLOWER2_* values.
 *
 * If a servo buzzes or strains at an end, back off a little. Do not push it into a hard stop.
 */
@TeleOp(name = "Flower Servo Tuner", group = "BioBuzz")
public class FlowerServoTuner extends OpMode {

    private static final double[] STEPS = {0.001, 0.005, 0.01, 0.05};
    /** How fast the left stick sweeps the servo, position units per second at full push. */
    private static final double STICK_RATE_PER_SEC = 0.25;

    private final Servo[] servos = new Servo[2];
    private final double[] position = {FLOWER_REST, FLOWER2_REST};
    private final double[] savedRest = {FLOWER_REST, FLOWER2_REST};
    private final double[] savedActive = {FLOWER_ACTIVE, FLOWER2_ACTIVE};
    private int selected = 0;
    private int stepIndex = 2; // starts at step 0.01

    private long lastLoopNs;
    private boolean lastUp, lastDown, lastLeft, lastRight;
    private boolean lastCross, lastSquare, lastCircle, lastTriangle;
    private boolean lastLb, lastRb, lastSwitch, lastRt, lastLt;

    @Override
    public void init() {
        servos[0] = hardwareMap.get(Servo.class, FLOWER);
        servos[1] = hardwareMap.get(Servo.class, FLOWER2);
        for (int i = 0; i < 2; i++) servos[i].setPosition(position[i]);
        lastLoopNs = System.nanoTime();

        telemetry.addLine("Flower Servo Tuner ready. Starting at the current config positions.");
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
        boolean sw = gamepad1.options || gamepad1.share;
        boolean rt = gamepad1.right_trigger > 0.5, lt = gamepad1.left_trigger > 0.5;

        if (sw && !lastSwitch) selected = 1 - selected;
        int s = selected;

        if (right && !lastRight) stepIndex = Math.min(stepIndex + 1, STEPS.length - 1);
        if (left && !lastLeft) stepIndex = Math.max(stepIndex - 1, 0);

        if (up && !lastUp) position[s] += STEPS[stepIndex];
        if (down && !lastDown) position[s] -= STEPS[stepIndex];

        // Slow sweep with the stick (deadzone so it does not drift)
        double stick = -gamepad1.left_stick_y;
        if (Math.abs(stick) > 0.1) position[s] += stick * STICK_RATE_PER_SEC * dt;

        if (cross && !lastCross) savedActive[s] = round3(position[s]);
        if (square && !lastSquare) savedRest[s] = round3(position[s]);
        if (circle && !lastCircle) position[s] = savedActive[s];
        if (triangle && !lastTriangle) position[s] = savedRest[s];
        if (rb && !lastRb) position[s] = 0.0;
        if (lb && !lastLb) position[s] = 1.0;
        if (rt && !lastRt) for (int i = 0; i < 2; i++) position[i] = savedActive[i];
        if (lt && !lastLt) for (int i = 0; i < 2; i++) position[i] = savedRest[i];

        lastUp = up; lastDown = down; lastLeft = left; lastRight = right;
        lastCross = cross; lastSquare = square; lastCircle = circle; lastTriangle = triangle;
        lastLb = lb; lastRb = rb; lastSwitch = sw; lastRt = rt; lastLt = lt;

        for (int i = 0; i < 2; i++) {
            position[i] = Range.clip(position[i], 0.0, 1.0);
            servos[i].setPosition(position[i]);
        }

        /* ---------- TELEMETRY ---------- */
        telemetry.addData(">>> TUNING", s == 0 ? "Flower1 (" + FLOWER + ")" : "Flower2 (" + FLOWER2 + ")");
        telemetry.addData("Position", "%.3f", position[s]);
        telemetry.addData("Step", "%.3f   (dpad right = bigger, left = smaller)", STEPS[stepIndex]);
        telemetry.addLine();
        telemetry.addData("Flower1 now", "%.3f   saved REST %.3f / ACTIVE %.3f", position[0], savedRest[0], savedActive[0]);
        telemetry.addData("Flower2 now", "%.3f   saved REST %.3f / ACTIVE %.3f", position[1], savedRest[1], savedActive[1]);
        telemetry.addLine();
        telemetry.addLine("Paste into BioBuzzConfig:");
        telemetry.addLine(String.format("  FLOWER_REST    = %.3f;", savedRest[0]));
        telemetry.addLine(String.format("  FLOWER_ACTIVE  = %.3f;", savedActive[0]));
        telemetry.addLine(String.format("  FLOWER2_REST   = %.3f;", savedRest[1]));
        telemetry.addLine(String.format("  FLOWER2_ACTIVE = %.3f;", savedActive[1]));
        telemetry.addLine();
        telemetry.addLine("options = switch servo   triggers = both servos REST/ACTIVE");
        telemetry.update();
    }

    private static double round3(double v) {
        return Math.round(v * 1000.0) / 1000.0;
    }
}
