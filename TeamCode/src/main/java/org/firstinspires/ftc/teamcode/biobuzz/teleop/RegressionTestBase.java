package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import java.util.ArrayList;
import java.util.List;

/**
 * Shooter regression data collection. Use {@link RegressionTestBlue} or {@link RegressionTestRed}.
 *
 * Drive to a spot, let the turret aim at the PRIMARY goal (never switches to the secondary), shoot with the
 * target velocity you want, and write down (distance, velocity) for the shots that go in. Telemetry (driver
 * station AND Panels) reports the distance from the turret to the goal every loop.
 *
 * Same hardware logic as {@link BioBuzzTeleOpV2Base}: the flywheel ALWAYS runs on the hub velocity PIDF
 * (SHOOTER_P / I / D / F from BioBuzzConfig, currently P=400 F=13.85), the turret auto-aims, and
 * gamepad2 right trigger (hold) = shoot (opens the blocker, intake + transfer at PIDF_INTAKE_TRANSFER_POWER).
 *
 * TARGET VELOCITY: starts at SHOOTER_FIXED_VELOCITY. Change it with gamepad2 dpad up / down (+- VELOCITY_STEP)
 * or type it into Panels (Configurables -> "RegressionTestBase" -> TARGET_VELOCITY / VELOCITY_STEP).
 * Panels edits and the dpad both change the same number.
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose, cross = toggle flower
 * gamepad2: right trigger (hold) = shoot, right bumper = intake + transfer, left bumper = intake only,
 *           cross = reverse both, triangle (hold) = open blocker,
 *           dpad up / down = target velocity + / - VELOCITY_STEP,
 *           dpad left / right = turret trim, options = declare the turret's current position straight forward,
 *           square = SAVE a sample (distance, target velocity) to the list below, share = clear the list
 *
 * Panels graph keys: "Target", "Actual", "Error", "Distance".
 */
@Configurable
public abstract class RegressionTestBase extends OpMode {

    // ================= EDITABLE IN PANELS =================
    /** Flywheel target (ticks/s). Starts at SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET. */
    public static double TARGET_VELOCITY = SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET;
    /** How much one dpad up / down press changes TARGET_VELOCITY. */
    public static double VELOCITY_STEP = 20;
    // ======================================================

    private static final double MAX_TARGET = MAX_SHOOTER_VELOCITY;
    private static final int MAX_SAMPLES = 15;

    private final BioBuzzAlliance alliance;

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker, flower;
    private BioBuzzTurret turret;
    private TelemetryManager panels;

    private boolean flowerActive = false;
    private boolean lastG1Cross, lUp, lDown, lSquare, lShare;
    private boolean firstLoop = true;

    private Pose lastPose = null;
    private long lastNanos = 0;
    private double velX = 0, velY = 0;

    private final List<String> samples = new ArrayList<>();

    protected RegressionTestBase(BioBuzzAlliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(BioBuzzPose.hasPoseFromAuto() ? BioBuzzPose.getPose() : alliance.startPose);

        shooter  = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake   = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker  = hardwareMap.get(Servo.class, BLOCKER);
        flower   = hardwareMap.get(Servo.class, FLOWER);

        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);

        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F));

        blocker.setPosition(BLOCKER_BLOCK);
        flower.setPosition(FLOWER_REST);

        turret = new BioBuzzTurret(hardwareMap, alliance);
        turret.setSecondaryGoal(false); // primary goal only
        Integer autoTicks = BioBuzzPose.getTurretTicks();
        if (BioBuzzPose.hasPoseFromAuto() && autoTicks != null) {
            turret.setEncoderOffset(autoTicks);
        }

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addData("Status", "Initialized (" + alliance + ", regression, primary goal)");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        if (firstLoop) { // let the localizer settle
            firstLoop = false;
            return;
        }

        if (gamepad1.triangle) {
            follower.setPose(alliance.resetPose);
        }
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

        long nowNanos = System.nanoTime();
        Pose pose = follower.pose();
        if (lastPose != null) {
            double dt = (nowNanos - lastNanos) / 1e9;
            if (dt > 0.005) {
                velX = (pose.x() - lastPose.x()) / dt;
                velY = (pose.y() - lastPose.y()) / dt;
                lastPose = pose;
                lastNanos = nowNanos;
            }
        } else {
            lastPose = pose;
            lastNanos = nowNanos;
        }

        /* ---------- TARGET VELOCITY (dpad or Panels) ---------- */
        boolean up = gamepad2.dpad_up, down = gamepad2.dpad_down;
        if (up && !lUp) TARGET_VELOCITY += VELOCITY_STEP;
        if (down && !lDown) TARGET_VELOCITY -= VELOCITY_STEP;
        lUp = up; lDown = down;
        TARGET_VELOCITY = Math.max(0, Math.min(MAX_TARGET, TARGET_VELOCITY));

        /* ---------- TURRET: primary goal only ---------- */
        turret.setSecondaryGoal(false);
        double trimDir = (gamepad2.dpad_left ? 1 : 0) - (gamepad2.dpad_right ? 1 : 0);
        turret.adjustTrim(trimDir * TURRET_TRIM_RATE_DEG_PER_SEC, 0.02);
        if (gamepad2.options) {
            turret.resetEncoder();
        }
        turret.aimAtGoal(pose, velX, velY);
        double distance = turret.getDistanceToGoal(pose);

        /* ---------- SHOOTER (always running) + SHOOT ---------- */
        double velocity = shooter.getVelocity();
        shooter.setVelocity(TARGET_VELOCITY);

        boolean shooting = gamepad2.right_trigger > 0.1;
        blocker.setPosition(shooting || gamepad2.triangle ? BLOCKER_OPEN : BLOCKER_BLOCK);

        if (gamepad1.cross && !lastG1Cross) flowerActive = !flowerActive;
        lastG1Cross = gamepad1.cross;
        flower.setPosition(flowerActive ? FLOWER_ACTIVE : FLOWER_REST);

        double intakePower = 0, transferPower = 0;
        if (gamepad2.cross) {
            intakePower = INTAKE_REVERSE_POWER;
            transferPower = TRANSFER_REVERSE_POWER;
        } else if (shooting) {
            intakePower = PIDF_INTAKE_TRANSFER_POWER;
            transferPower = PIDF_INTAKE_TRANSFER_POWER;
        } else if (gamepad2.left_bumper) {
            intakePower = INTAKE_IN_POWER;
        } else if (gamepad2.right_bumper) {
            intakePower = INTAKE_IN_POWER;
            transferPower = TRANSFER_IN_POWER;
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        /* ---------- SAMPLES ---------- */
        boolean square = gamepad2.square, share = gamepad2.share;
        if (square && !lSquare) {
            samples.add(String.format("%.1f in -> %.0f", distance, TARGET_VELOCITY));
            if (samples.size() > MAX_SAMPLES) samples.remove(0);
        }
        if (share && !lShare) samples.clear();
        lSquare = square; lShare = share;

        /* ---------- TELEMETRY ---------- */
        boolean ready = Math.abs(TARGET_VELOCITY - velocity) < SHOOTER_READY_TOLERANCE;
        telemetry.addData("Alliance", alliance + " (primary goal only)");
        telemetry.addData("DISTANCE TO GOAL (in)", "%.2f", distance);
        telemetry.addData("Target / actual (ticks/s)", "%.0f / %.0f", TARGET_VELOCITY, velocity);
        telemetry.addData("Velocity error", "%.0f  %s", TARGET_VELOCITY - velocity, ready ? "READY" : "recovering");
        telemetry.addData("PIDF", "P=%s I=%s D=%s F=%s", SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        telemetry.addData("Turret angle / target / trim", "%.1f / %.1f / %.1f", turret.getAngle(), turret.getTargetAngle(), turret.getTrim());
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Shooting", shooting ? "YES" : "no");
        telemetry.addData("Saved samples (square = save, share = clear)", samples.isEmpty() ? "-" : "");
        for (int i = 0; i < samples.size(); i++) telemetry.addData("  " + (i + 1), samples.get(i));
        telemetry.update();

        panels.addData("Target", TARGET_VELOCITY);
        panels.addData("Actual", velocity);
        panels.addData("Error", TARGET_VELOCITY - velocity);
        panels.addData("Distance", distance);
        panels.addData("Flywheel", ready ? "AT SPEED" : "recovering");
        panels.update();
    }

    @Override
    public void stop() {
        turret.stop();
        shooter.setVelocity(0);
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
