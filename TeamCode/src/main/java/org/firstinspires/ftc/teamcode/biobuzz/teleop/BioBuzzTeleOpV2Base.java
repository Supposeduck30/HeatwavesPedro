package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The main BioBuzz TeleOp ("BioBuzz TeleOp Blue" / "BioBuzz TeleOp Red"). Use {@link BioBuzzTeleOpV2Blue} or
 * {@link BioBuzzTeleOpV2Red}.
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose, cross = toggle flower,
 *           share DURING INIT = start from the default start pose instead of where the last auto ended
 *           (the auto pose is only used once: a second teleop in a row always starts from the default)
 * gamepad2: the flywheel ALWAYS runs (hub PIDF at the distance regression target) from the moment the OpMode starts,
 *           right trigger (hold) = SHOOT: opens the blocker and runs intake + transfer at PIDF_INTAKE_TRANSFER_POWER,
 *           right bumper = intake + transfer (blocker stays closed),
 *           left bumper = intake only, cross = reverse both, triangle (hold) = open blocker,
 *           circle = RE-HOME the turret on the magnet switch (see HOMING below; circle again = cancel),
 *           options = toggle primary/secondary goal (ignored when the alliance auto-picks by y),
 *           dpad left/right = turret trim (tap = one step, hold = continuous),
 *           dpad up = LOCK toggle: freeze the turret where it is (stops auto-aim); tap again to resume aiming,
 *           dpad down = zero turret (current position = straight forward)
 * The turret auto-aims at the goal every loop (unless locked or homing).
 *
 * HOMING (gamepad2 circle): goal tracking is paused and the turret runs on raw power. It spins CLOCKWISE fast
 * (TURRET_HOME_FAST_POWER) until the magnet switch (REV Touch Sensor TURRET_LIMIT) reads pressed, backs off
 * counter-clockwise TURRET_HOME_BACKOFF_DEG, then comes back in clockwise slowly (TURRET_HOME_POWER) so it always hits
 * the same (left) edge. There it is told it is at TURRET_LIMIT_ANGLE_DEG, which wipes out the drift from belt/gear
 * skipping, and tracking resumes. (If it starts on the magnet it skips the fast part and just backs off first.)
 * No magnet within HOMING_TIMEOUT_MS = the turret stops and the screen shows "TURRET HOMING FAILED" (circle = retry).
 * If the switch is missing from the robot config, circle does nothing.
 *
 * Shooter uses the hub velocity PIDF (SHOOTER_P / I / D / F). The target comes from the distance-to-goal regression
 * (velA / velB / velC in {@link BioBuzzAlliance}), capped at MAX_SHOOTER_VELOCITY. Tunables: {@link BioBuzzConfig}.
 */
public abstract class BioBuzzTeleOpV2Base extends OpMode {

    // ================= MAGNET SWITCH HOMING =================
    // Switch name, home angle (TURRET_LIMIT_ANGLE_DEG), and powers live in BioBuzzConfig.
    /** true = also home once automatically when the OpMode starts. false = only when gamepad2 circle is pressed. */
    public static final boolean HOME_ON_START = false;
    /** Give up (TURRET HOMING FAILED) if the magnet isn't found this long after homing starts. */
    public static final long HOMING_TIMEOUT_MS = 15000;

    private enum Homing { NO_SWITCH, IDLE, FAST_SEEK, BACKING_OFF, SEEKING, HOMED, FAILED }

    private TouchSensor magSwitch; // configured as "REV Touch Sensor"; isPressed() = magnet detected
    private DcMotorEx turretMotor;
    private Homing homing = Homing.NO_SWITCH;
    private long homingStartMs;
    private int backoffRefTicks; // back off TURRET_HOME_BACKOFF_DEG from here (Integer.MIN_VALUE = not set yet)
    private double lastHomeCorrectionDeg = Double.NaN;
    private boolean lastGamepad2Options = false;
    // ========================================================

    private final BioBuzzAlliance alliance;

    private Pose savedAutoPose;          // null = no auto ran since the last teleop
    private Integer savedAutoTurretTicks;
    private boolean useAutoPose;
    private boolean lastGamepad1Share = false;

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker, flower;

    private boolean flowerActive = false;
    private boolean lastGamepad1Cross = false;
    private boolean lastGamepad2Circle = false;
    private boolean lastGamepad2DpadLeft = false, lastGamepad2DpadRight = false;
    private long dpadLeftPressNanos = 0, dpadRightPressNanos = 0, lastTrimNanos = 0;
    private boolean lastGamepad2DpadUp = false;
    private boolean turretLocked = false;
    private boolean firstLoop = true;

    private BioBuzzTurret turret;

    // Field-frame velocity from pose differences (used for shot leading)
    private Pose lastPose = null;
    private long lastNanos = 0;
    private double velX = 0, velY = 0;

    protected BioBuzzTeleOpV2Base(BioBuzzAlliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Where auto ended (if an auto ran since the last teleop). Used by default; gamepad1 share during init
        // switches to the alliance's default start pose for testing. Applied by applyStartChoice().
        savedAutoPose = BioBuzzPose.getPose();
        savedAutoTurretTicks = BioBuzzPose.getTurretTicks();
        useAutoPose = savedAutoPose != null;

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
        applyStartChoice();

        // Magnet switch (REV Touch Sensor). Missing from the config = no homing, no crash.
        try {
            magSwitch = hardwareMap.tryGet(TouchSensor.class, TURRET_LIMIT);
        } catch (Exception e) {
            magSwitch = null;
        }
        if (magSwitch != null) {
            turretMotor = hardwareMap.get(DcMotorEx.class, TURRET); // the same motor BioBuzzTurret drives
            homing = Homing.IDLE;
        }

    }

    @Override
    public void init_loop() {
        // gamepad1 share = switch between "where auto ended" and the default start pose (only if an auto saved one)
        if (gamepad1.share && !lastGamepad1Share && savedAutoPose != null) {
            useAutoPose = !useAutoPose;
            applyStartChoice();
        }
        lastGamepad1Share = gamepad1.share;

        telemetry.addData("Status", "Initialized (" + alliance + ")");
        if (savedAutoPose == null) {
            telemetry.addData("Start pose", "DEFAULT start pose (no auto ran since the last teleop)");
        } else {
            telemetry.addData("Start pose", useAutoPose
                    ? String.format("WHERE AUTO ENDED (%.1f, %.1f)  - gamepad1 share = use default instead",
                            savedAutoPose.x(), savedAutoPose.y())
                    : "DEFAULT start pose  - gamepad1 share = use where auto ended instead");
        }
        telemetry.addData("Mag switch", magSwitch != null ? "found \"" + TURRET_LIMIT + "\" (gamepad2 circle = home)"
                : "NOT FOUND \"" + TURRET_LIMIT + "\" (add it as a REV Touch Sensor) - homing off");
        telemetry.update();
    }

    @Override
    public void start() {
        // The auto pose is used once: the next teleop starts from the default unless another auto runs first.
        BioBuzzPose.clear();
        // Motors can't move during init, so start-homing (if on) begins as soon as the OpMode starts.
        if (HOME_ON_START) beginHoming();
    }

    /**
     * Sets the robot pose and turret angle from the init choice. Default = alliance start pose with the turret
     * straight forward; auto = the pose and turret ticks the last auto saved.
     */
    private void applyStartChoice() {
        if (useAutoPose && savedAutoPose != null) {
            follower.setPose(savedAutoPose);
            turret.setEncoderOffset(savedAutoTurretTicks != null ? savedAutoTurretTicks : 0);
        } else {
            follower.setPose(alliance.startPose);
            turret.setEncoderOffset(0); // straight forward (encoder was just reset)
        }
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

        // Goal: automatic by the robot's y when the alliance has a split, else gamepad2 options toggles it
        if (!Double.isNaN(alliance.autoGoalSplitY)) {
            turret.setSecondaryGoal(alliance.useSecondaryGoalAt(pose.y()));
        } else if (gamepad2.options && !lastGamepad2Options) {
            turret.toggleGoal();
        }
        lastGamepad2Options = gamepad2.options;

        // Turret: auto-aim at the goal (A/B picked by the robot's side of the halfway mark). dpad left/right = trim
        // Drift correction (left = CCW, right = CW):
        //   each TAP nudges the aim TURRET_TRIM_STEP_DEG instantly (spam-tap to move fast),
        //   HOLDING past TURRET_TRIM_HOLD_DELAY_SEC keeps moving it at TURRET_TRIM_HOLD_RATE_DEG_PER_SEC.
        double loopDt = lastTrimNanos == 0 ? 0 : Math.min((nowNanos - lastTrimNanos) / 1e9, 0.1);
        lastTrimNanos = nowNanos;
        if (gamepad2.dpad_left && !lastGamepad2DpadLeft) {
            turret.adjustTrim(TURRET_TRIM_STEP_DEG, 1.0);
            dpadLeftPressNanos = nowNanos;
        } else if (gamepad2.dpad_left && (nowNanos - dpadLeftPressNanos) / 1e9 >= TURRET_TRIM_HOLD_DELAY_SEC) {
            turret.adjustTrim(TURRET_TRIM_HOLD_RATE_DEG_PER_SEC, loopDt);
        }
        if (gamepad2.dpad_right && !lastGamepad2DpadRight) {
            turret.adjustTrim(-TURRET_TRIM_STEP_DEG, 1.0);
            dpadRightPressNanos = nowNanos;
        } else if (gamepad2.dpad_right && (nowNanos - dpadRightPressNanos) / 1e9 >= TURRET_TRIM_HOLD_DELAY_SEC) {
            turret.adjustTrim(-TURRET_TRIM_HOLD_RATE_DEG_PER_SEC, loopDt);
        }
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

        // Re-home button (gamepad2 circle): start homing, or cancel it if it's already running
        if (gamepad2.circle && !lastGamepad2Circle && magSwitch != null) {
            if (isHoming()) {
                turretMotor.setPower(0);
                homing = Homing.IDLE;
            } else {
                turretLocked = false; // homing overrides the lock
                beginHoming();
            }
        }
        lastGamepad2Circle = gamepad2.circle;

        // While homing, the turret ignores goal tracking and runs on raw power.
        if (updateHoming()) {
            if (turretLocked) {
                turret.hold();
            } else {
                turret.aimAtGoal(pose, 0, 0); // no shot leading (same as auto)
            }
        }

        // Shooter: the flywheel always runs at the PIDF target
        double velocity = shooter.getVelocity();
        double distance = turret.getDistanceToGoal(pose);
        double shooterTarget = Math.max(0, Math.min(MAX_SHOOTER_VELOCITY,
                alliance.shooterVelocity(distance) + PIDF_VELOCITY_OFFSET));
        shooter.setVelocity(shooterTarget);

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
        telemetry.addData("Turret", isHoming() ? "HOMING (gamepad2 circle = cancel)"
                : (turretLocked ? "LOCKED (gamepad2 dpad up to unlock)" : "auto-aim"));
        if (homing == Homing.FAILED) {
            telemetry.addData("!!!", "TURRET HOMING FAILED (no magnet in " + HOMING_TIMEOUT_MS + " ms). gamepad2 circle = try again");
        }
        telemetry.addData("Homing state", homing);
        telemetry.addData("Mag switch", magSwitch == null ? "-" : (magnetDetected() ? "MAGNET" : "clear"));
        telemetry.addData("Last home correction (deg)", Double.isNaN(lastHomeCorrectionDeg) ? "-"
                : String.format("%.2f (drift that was removed)", lastHomeCorrectionDeg));
        telemetry.addData("Turret angle / target", "%.1f / %.1f", turret.getAngle(), turret.getTargetAngle());
        telemetry.addData("Turret ticks", turret.getRawTicks());
        telemetry.addData("Turret trim (deg)", "%.1f", turret.getTrim());
        telemetry.addData("Goal", turret.isTargetingSecondaryGoal() ? "SECONDARY" : "PRIMARY");
        telemetry.addData("Distance to goal (in)", "%.1f", distance);
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Shooter target / actual", "%.0f / %.0f", shooterTarget, velocity);
        telemetry.addData("Blocker", shooting || gamepad2.triangle ? "OPEN" : "BLOCKING");
        telemetry.addData("Shooting", shooting ? "YES" : "no");
        telemetry.update();
    }

    // ================= MAGNET HOMING =================
    private boolean magnetDetected() {
        return magSwitch != null && magSwitch.isPressed();
    }

    private boolean isHoming() {
        return homing == Homing.FAST_SEEK || homing == Homing.BACKING_OFF || homing == Homing.SEEKING;
    }

    private void beginHoming() {
        if (magSwitch == null) return;
        homingStartMs = System.currentTimeMillis();
        backoffRefTicks = Integer.MIN_VALUE;
        homing = magnetDetected() ? Homing.BACKING_OFF : Homing.FAST_SEEK;
    }

    /** Power in the angle frame (+ = counter-clockwise, - = clockwise); applies the turret's power sign. */
    private void turretAnglePower(double p) {
        turretMotor.setPower(TURRET_POWER_SIGN * p);
    }

    /** Runs one step of homing. Returns true when the turret may track the goal (not homing). */
    private boolean updateHoming() {
        if (!isHoming()) {
            if (homing == Homing.FAILED) {
                turretMotor.setPower(0);
                return false; // stay stopped until circle is pressed again
            }
            return true; // NO_SWITCH / IDLE / HOMED: normal tracking
        }
        if (System.currentTimeMillis() - homingStartMs > HOMING_TIMEOUT_MS) {
            turretMotor.setPower(0);
            homing = Homing.FAILED;
            return false;
        }
        if (homing == Homing.FAST_SEEK) {
            // 1) Spin clockwise fast until the magnet shows up (rough position only: too fast to be exact).
            if (!magnetDetected()) {
                turretAnglePower(-Math.abs(TURRET_HOME_FAST_POWER));
                return false;
            }
            backoffRefTicks = turretMotor.getCurrentPosition();
            homing = Homing.BACKING_OFF;
        }
        if (homing == Homing.BACKING_OFF) {
            // 2) Go counter-clockwise off the magnet and TURRET_HOME_BACKOFF_DEG past where it was found (or past where
            // it cleared, if homing started on it), so the slow clockwise seek always comes into the same (left) edge.
            int ticks = turretMotor.getCurrentPosition();
            if (backoffRefTicks == Integer.MIN_VALUE && !magnetDetected()) backoffRefTicks = ticks;
            boolean farEnough = backoffRefTicks != Integer.MIN_VALUE && !magnetDetected()
                    && Math.abs(ticks - backoffRefTicks) >= TURRET_HOME_BACKOFF_DEG * TURRET_TICKS_PER_DEGREE;
            if (!farEnough) {
                turretAnglePower(Math.abs(TURRET_HOME_BACKOFF_POWER));
                return false;
            }
            homing = Homing.SEEKING;
        }
        // 3) SEEKING: slow raw power clockwise until the magnet is detected = the exact home edge
        if (magnetDetected()) {
            turretMotor.setPower(0);
            double before = turret.getAngle();
            int offset = (int) Math.round((TURRET_LIMIT_ANGLE_DEG - TURRET_START_ANGLE_DEG) * TURRET_TICKS_PER_DEGREE)
                    - turretMotor.getCurrentPosition();
            turret.setEncoderOffset(offset); // the angle now reads TURRET_LIMIT_ANGLE_DEG
            lastHomeCorrectionDeg = turret.getAngle() - before;
            homing = Homing.HOMED;
            return true;
        }
        turretAnglePower(-Math.abs(TURRET_HOME_POWER));
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
