package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BioBuzz V3 = V2 plus MAGNET-SWITCH HOMING of the turret (V2 is untouched). Use {@link BioBuzzTeleOpV3Blue} or
 * {@link BioBuzzTeleOpV3Red}.
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose, cross = toggle flower
 * gamepad2: the flywheel ALWAYS runs (hub PIDF at SHOOTER_FIXED_VELOCITY) from the moment the OpMode starts,
 *           right trigger (hold) = SHOOT: opens the blocker and runs intake + transfer at PIDF_INTAKE_TRANSFER_POWER,
 *           right bumper = intake + transfer (blocker stays closed),
 *           left bumper = intake only, cross = reverse both, triangle (hold) = open blocker,
 *           circle = toggle primary/secondary goal (ignored when the alliance auto-picks by y),
 *           dpad left/right = turret trim,
 *           dpad up = LOCK toggle: freeze the turret where it is (stops auto-aim); tap again to resume aiming,
 *           dpad down = zero turret (current position = straight forward),
 *           options = RE-HOME the turret on the magnet switch (does nothing unless the switch is enabled and found)
 * The turret auto-aims at the goal every loop.
 *
 * HOMING (off until USE_TURRET_MAG_SWITCH = true AND the switch is found in the robot config): when the OpMode starts
 * the turret rotates slowly CLOCKWISE until the magnet is detected (if it starts on the magnet it first backs off
 * counter-clockwise), stops, declares its angle to be HOME_ANGLE_DEG, and only THEN starts tracking the goal.
 * If it doesn't find the magnet within HOMING_TIMEOUT_MS the turret stops and the screen shows "TURRET HOMING FAILED".
 * With homing off (flag false or switch missing) this runs exactly like V2.
 * The turret has no angle limits (it spins freely), so tracking is not clamped.
 *
 * Shooter uses the hub velocity PIDF (SHOOTER_P / I / D / F) at SHOOTER_FIXED_VELOCITY. Tunables: {@link BioBuzzConfig}.
 */
public abstract class BioBuzzTeleOpV3Base extends OpMode {

    // ================= MAGNET SWITCH HOMING (the switch is NOT installed yet) =================
    /** Master switch. false = V3 behaves exactly like V2. */
    public static final boolean USE_TURRET_MAG_SWITCH = false;
    /** Name of the switch (a digital channel) in the robot config. */
    public static final String TURRET_MAG_SWITCH_NAME = "turret_mag";
    /** The turret angle (deg, same frame as BioBuzzTurret: 90 = straight forward) at the CLOCKWISE edge of the magnet. */
    public static final double HOME_ANGLE_DEG = 43.52;
    /** What getState() reads while the magnet is detected (a REV magnetic switch pulls the line LOW = false). */
    public static final boolean MAG_ACTIVE_STATE = false;
    /** Slow turret power while looking for the magnet (0..1; the direction is handled for you). */
    public static final double HOMING_POWER = 0.15;
    /** Give up (TURRET HOMING FAILED) if the magnet isn't found this long after homing starts. */
    public static final long HOMING_TIMEOUT_MS = 10000;
    // ==========================================================================================

    private enum Homing { DISABLED, PENDING, BACKING_OFF, SEEKING, HOMED, FAILED }

    private final BioBuzzAlliance alliance;

    private DigitalChannel magSwitch;
    private DcMotorEx turretMotor;
    private Homing homing = Homing.DISABLED;
    private long homingStartMs;
    private boolean lastGamepad2Options = false;

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker, flower;

    private boolean flowerActive = false;
    private boolean lastGamepad1Cross = false;
    private boolean lastGamepad2Circle = false;
    private boolean lastGamepad2DpadLeft = false, lastGamepad2DpadRight = false;
    private boolean lastGamepad2DpadUp = false;
    private boolean turretLocked = false;
    private boolean firstLoop = true;

    private BioBuzzTurret turret;

    // Field-frame velocity from pose differences (used for shot leading)
    private Pose lastPose = null;
    private long lastNanos = 0;
    private double velX = 0, velY = 0;

    protected BioBuzzTeleOpV3Base(BioBuzzAlliance alliance) {
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
        Integer autoTicks = BioBuzzPose.getTurretTicks();
        if (BioBuzzPose.hasPoseFromAuto() && autoTicks != null) {
            turret.setEncoderOffset(autoTicks); // turret stayed where auto left it
        }

        // Magnet switch: optional. Off, or missing from the config = no homing, no crash.
        magSwitch = null;
        if (USE_TURRET_MAG_SWITCH) {
            try {
                magSwitch = hardwareMap.tryGet(DigitalChannel.class, TURRET_MAG_SWITCH_NAME);
                if (magSwitch != null) magSwitch.setMode(DigitalChannel.Mode.INPUT);
            } catch (Exception e) {
                magSwitch = null;
            }
        }
        if (magSwitch != null) {
            turretMotor = hardwareMap.get(DcMotorEx.class, TURRET); // the same motor BioBuzzTurret drives
            homing = Homing.PENDING;
        } else {
            homing = Homing.DISABLED;
        }

        telemetry.addData("Status", "Initialized (" + alliance + ")");        telemetry.addData("Mag switch", !USE_TURRET_MAG_SWITCH ? "OFF (USE_TURRET_MAG_SWITCH = false)"
                : (magSwitch != null ? "found \"" + TURRET_MAG_SWITCH_NAME + "\"" : "NOT FOUND \"" + TURRET_MAG_SWITCH_NAME + "\""));
        telemetry.update();
    }

    @Override
    public void start() {
        // Motors can't move during init, so homing begins as soon as the OpMode starts.
        if (homing == Homing.PENDING) beginHoming();
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

        // Velocity estimate (field frame) for shot leading
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

        // Goal: automatic by the robot's y when the alliance has a split, else gamepad2 circle toggles it
        if (!Double.isNaN(alliance.autoGoalSplitY)) {
            turret.setSecondaryGoal(alliance.useSecondaryGoalAt(pose.y()));
        } else if (gamepad2.circle && !lastGamepad2Circle) {
            turret.toggleGoal();
        }
        lastGamepad2Circle = gamepad2.circle;

        // Turret trim / zero (same as V2)
        // Drift correction: each tap nudges the aim TURRET_TRIM_STEP_DEG (left = CCW, right = CW)
        if (gamepad2.dpad_left && !lastGamepad2DpadLeft)   turret.adjustTrim(TURRET_TRIM_STEP_DEG, 1.0);
        if (gamepad2.dpad_right && !lastGamepad2DpadRight) turret.adjustTrim(-TURRET_TRIM_STEP_DEG, 1.0);
        lastGamepad2DpadLeft = gamepad2.dpad_left;
        lastGamepad2DpadRight = gamepad2.dpad_right;
        if (gamepad2.dpad_down) {
            turret.resetEncoder(); // current physical position = straight forward
        }
        // dpad up = LOCK toggle (worst case): freeze the turret where it is and stop auto-aiming. Tap again to resume.
        if (gamepad2.dpad_up && !lastGamepad2DpadUp) {
            turretLocked = !turretLocked;
            if (turretLocked) turret.lockHere();
        }
        lastGamepad2DpadUp = gamepad2.dpad_up;

        // Re-home button (gamepad2 options)
        if (gamepad2.options && !lastGamepad2Options && magSwitch != null) {
            beginHoming();
        }
        lastGamepad2Options = gamepad2.options;

        // Track the goal only once homed (or when homing is off, exactly like V2).
        if (turretLocked) {
            turret.hold(); // locked overrides homing and aiming
        } else if (updateHoming()) {
            turret.aimAtGoal(pose, velX, velY);
        }

        // Shooter: the flywheel always runs at the PIDF target
        double velocity = shooter.getVelocity();
        shooter.setVelocity(SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET);

        // Right trigger (hold) = shoot: open the blocker and feed at the shooter feed power (same as Shooter Graph Test)
        boolean shooting = gamepad2.right_trigger > 0.1;
        blocker.setPosition(shooting || gamepad2.triangle ? BLOCKER_OPEN : BLOCKER_BLOCK);

        // Flower: gamepad1 cross toggles
        if (gamepad1.cross && !lastGamepad1Cross) {
            flowerActive = !flowerActive;
        }
        lastGamepad1Cross = gamepad1.cross;
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

        telemetry.addData("Alliance", alliance);
        telemetry.addData("Turret", turretLocked ? "LOCKED (gamepad2 dpad up to unlock)" : "auto-aim");
        if (homing == Homing.FAILED) {
            telemetry.addData("!!!", "TURRET HOMING FAILED (no magnet in " + HOMING_TIMEOUT_MS + " ms). gamepad2 options = try again");
        }
        telemetry.addData("Mag switch found", magSwitch != null);
        telemetry.addData("Homed / state", "%s / %s",
                homing == Homing.HOMED ? "YES" : (homing == Homing.DISABLED ? "n/a (homing off)" : "no"), homing);
        telemetry.addData("Mag switch state", magSwitch == null ? "-"
                : (magnetDetected() ? "MAGNET" : "clear") + " (raw " + magSwitch.getState() + ")");
        telemetry.addData("Turret angle / target", "%.1f / %.1f", turret.getAngle(), turret.getTargetAngle());
        telemetry.addData("Turret ticks", turret.getRawTicks());
        telemetry.addData("Turret trim (deg)", "%.1f", turret.getTrim());
        telemetry.addData("Goal", turret.isTargetingSecondaryGoal() ? "SECONDARY" : "PRIMARY");
        telemetry.addData("Distance to goal (in)", "%.1f", turret.getDistanceToGoal(pose));
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Shooter target / actual", "%.0f / %.0f", SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET, velocity);
        telemetry.addData("Blocker", shooting || gamepad2.triangle ? "OPEN" : "BLOCKING");
        telemetry.addData("Shooting", shooting ? "YES" : "no");
        telemetry.update();
    }

    // ================= MAGNET HOMING =================
    private boolean magnetDetected() {
        return magSwitch != null && magSwitch.getState() == MAG_ACTIVE_STATE;
    }

    private void beginHoming() {
        if (magSwitch == null) return;
        homingStartMs = System.currentTimeMillis();
        homing = magnetDetected() ? Homing.BACKING_OFF : Homing.SEEKING;
    }

    /** Power in the angle frame (+ = counter-clockwise, - = clockwise); applies the turret's power sign. */
    private void turretAnglePower(double p) {
        turretMotor.setPower(TURRET_POWER_SIGN * p);
    }

    /** Runs one step of homing. Returns true when the turret may track the goal. */
    private boolean updateHoming() {
        switch (homing) {
            case DISABLED:
            case HOMED:
                return true;
            case PENDING:
                beginHoming();
                return false;
            case FAILED:
                turretMotor.setPower(0);
                return false;
            default:
                break;
        }
        if (System.currentTimeMillis() - homingStartMs > HOMING_TIMEOUT_MS) {
            turretMotor.setPower(0);
            homing = Homing.FAILED;
            return false;
        }
        if (homing == Homing.BACKING_OFF) {
            // Started on the magnet: back off counter-clockwise until clear, so the clockwise edge is what we home on.
            if (!magnetDetected()) {
                homing = Homing.SEEKING;
            } else {
                turretAnglePower(Math.abs(HOMING_POWER));
                return false;
            }
        }
        // SEEKING: clockwise until the magnet is detected
        if (magnetDetected()) {
            turretMotor.setPower(0);
            int offset = (int) Math.round((HOME_ANGLE_DEG - TURRET_START_ANGLE_DEG) * TURRET_TICKS_PER_DEGREE)
                    - turretMotor.getCurrentPosition();
            turret.setEncoderOffset(offset); // the angle now reads HOME_ANGLE_DEG
            homing = Homing.HOMED;
            return true;
        }
        turretAnglePower(-Math.abs(HOMING_POWER));
        return false;
    }

    @Override
    public void stop() {
        turret.stop();
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
