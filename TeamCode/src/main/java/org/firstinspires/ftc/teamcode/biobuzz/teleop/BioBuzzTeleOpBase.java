package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.biobuzz.shooter.ShooterControl;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The one BioBuzz TeleOp. Use the four OpModes in {@link BioBuzzTeleOps}; they only pick the alliance and
 * the shooter control ({@link org.firstinspires.ftc.teamcode.biobuzz.shooter.PIDFShooter} or
 * {@link org.firstinspires.ftc.teamcode.biobuzz.shooter.BangBangShooter}). Everything else is identical.
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose, dpad up = relocalize
 *           HOLD to drive to and hold a point: square = empty gate, circle = park,
 *           left bumper = shoot far, right bumper = shoot close
 *           cross = toggle flower
 * gamepad2: right bumper = intake + transfer,
 *           left bumper = intake only,
 *           cross = reverse both, triangle (hold) = open blocker,
 *           circle = toggle primary/secondary goal (RED picks it automatically by y, see BioBuzzAlliance.autoGoalSplitY), dpad down = reset the shot-test dip stats,
 *           dpad left / right = turret aim trim (left = counter-clockwise), options = declare the turret's
 *           current position to be straight forward (0 deg)
 *
 * The shooter runs at SHOOTER_FIXED_VELOCITY (or the distance regression if USE_FIXED_SHOOTER_VELOCITY is off).
 * How it is held there is up to the ShooterControl; its constants are in BioBuzzConfig.
 *
 * TURRET: {@link BioBuzzTurret} (infinite, no wrap-around) tracks the active goal from the Pedro pose, leading
 * the shot by the robot's velocity. Start it pointing straight forward at init (or press options to re-zero).
 * The goal distance used for the shooter regression is measured from the turret pivot.
 *
 * Tunables: {@link BioBuzzConfig} and {@link BioBuzzAlliance}.
 * Auto handoff (start pose): {@link BioBuzzPose}.
 */
public abstract class BioBuzzTeleOpBase extends OpMode {

    private enum HoldTarget { NONE, EMPTY_GATE, PARK, SHOOT_FAR, SHOOT_CLOSE }

    private final BioBuzzAlliance alliance;
    private final ShooterControl shooterControl;

    private Follower follower;

    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker;
    private Servo flower;
    private BioBuzzTurret turret;

    private boolean flowerActive = false;
    private boolean lastGamepad1Cross = false;

    // Shot test: how far the flywheel drops below target while shooting, and how long it takes to recover
    private boolean dipArmed = false;
    private double maxDip = 0, lowestSpeed = Double.MAX_VALUE;
    private boolean recovering = false;
    private long dipStartNs = 0;
    private double lastRecoverySec = -1, worstRecoverySec = -1;

    private HoldTarget activeHold = HoldTarget.NONE;
    private boolean lastGamepad2Circle = false;

    // Field-frame velocity from pose differences (used for shot leading)
    private Pose lastPose = null;
    private long lastNanos = 0;
    private double velX = 0, velY = 0;

    private boolean firstLoop = true;

    // Loop timing
    private long lastLoopNanos = 0;
    private double loopMs = 0, maxLoopMs = 0;

    protected BioBuzzTeleOpBase(BioBuzzAlliance alliance, ShooterControl shooterControl) {
        this.alliance = alliance;
        this.shooterControl = shooterControl;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        if (BioBuzzPose.hasPoseFromAuto()) {
            follower.setPose(BioBuzzPose.getPose());
        } else {
            follower.setPose(alliance.startPose);
        }

        shooter  = hardwareMap.get(DcMotorEx.class, SHOOTER);
        intake   = hardwareMap.get(DcMotor.class, INTAKE);
        transfer = hardwareMap.get(DcMotor.class, TRANSFER);
        blocker  = hardwareMap.get(Servo.class, BLOCKER);
        flower   = hardwareMap.get(Servo.class, FLOWER);

        shooter.setDirection(SHOOTER_DIR);
        intake.setDirection(INTAKE_DIR);
        transfer.setDirection(TRANSFER_DIR);

        shooterControl.init(shooter, hardwareMap);

        blocker.setPosition(BLOCKER_BLOCK);
        flower.setPosition(FLOWER_REST);

        // The turret encoder reads 0 now, which means "pointing straight forward" unless auto left it turned.
        turret = new BioBuzzTurret(hardwareMap, alliance);
        Integer autoTurretTicks = BioBuzzPose.getTurretTicks();
        if (autoTurretTicks != null) {
            turret.setEncoderOffset(autoTurretTicks);
        }
        telemetry.addData("Turret", autoTurretTicks != null
                ? "resumed from auto (" + autoTurretTicks + " ticks)"
                : "fresh start (point it straight forward)");

        telemetry.addData("Status", "Initialized (" + alliance + ", " + shooterControl.name() + ")");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();

        // The first update just lets the localizer settle.
        if (firstLoop) {
            firstLoop = false;
            return;
        }

        long nowNanos = System.nanoTime();

        if (lastLoopNanos != 0) {
            loopMs = (nowNanos - lastLoopNanos) / 1e6;
            maxLoopMs = Math.max(maxLoopMs, loopMs);
        }
        lastLoopNanos = nowNanos;

        if (gamepad1.triangle) {
            follower.setPose(alliance.resetPose);
        }

        // Relocalize: drive to the marked spot, press dpad up, and Pedro restarts odometry from there.
        if (gamepad1.dpad_up) {
            follower.setPose(alliance.relocalizePose);
        }

        /* ---------- HOLD POINTS / DRIVE ---------- */
        updateHold();
        if (activeHold == HoldTarget.NONE) {
            // manual() takes the follower out of HOLD mode, so only call it when not holding.
            follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        }

        /* ---------- VELOCITY ESTIMATE ---------- */
        Pose pose = follower.pose();
        if (lastPose != null) {
            double dt = (nowNanos - lastNanos) / 1e9;
            if (dt > 0.005) {
                velX = (pose.x() - lastPose.x()) / dt;
                velY = (pose.y() - lastPose.y()) / dt;
            }
        }
        lastPose = pose;
        lastNanos = nowNanos;

        /* ---------- GOAL TOGGLE ---------- */
        boolean circleNow = gamepad2.circle;
        if (!Double.isNaN(alliance.autoGoalSplitY)) {
            // Automatic: above the split y = secondary goal, otherwise primary (circle is ignored).
            turret.setSecondaryGoal(pose.y() > alliance.autoGoalSplitY);
        } else if (circleNow && !lastGamepad2Circle) {
            turret.toggleGoal();
        }
        lastGamepad2Circle = circleNow;

        /* ---------- TURRET ---------- */
        // TURRET DISABLED: motor is never driven. The turret object is still used for goal distance (shooter regression).
        // if (gamepad2.options) {
        //     turret.resetEncoder();
        // }
        // double trimRate = (gamepad2.dpad_left ? TURRET_TRIM_RATE_DEG_PER_SEC : 0)
        //         - (gamepad2.dpad_right ? TURRET_TRIM_RATE_DEG_PER_SEC : 0);
        // turret.adjustTrim(trimRate, loopMs / 1000.0);
        // turret.aimAtGoal(pose, velX, velY);

        /* ---------- BLOCKER ---------- */
        blocker.setPosition(gamepad2.triangle ? BLOCKER_OPEN : BLOCKER_BLOCK);

        /* ---------- FLOWER (gamepad1 cross toggles) ---------- */
        if (gamepad1.cross && !lastGamepad1Cross) {
            flowerActive = !flowerActive;
        }
        lastGamepad1Cross = gamepad1.cross;
        flower.setPosition(flowerActive ? FLOWER_ACTIVE : FLOWER_REST);

        /* ---------- SHOOTER ---------- */
        double distance = turret.getDistanceToGoal(pose);
        double targetVelocity = USE_FIXED_SHOOTER_VELOCITY
                ? fixedShooterVelocity()
                : Range.clip(alliance.shooterVelocity(distance) * SHOOTER_VELOCITY_SCALE, 0, MAX_SHOOTER_VELOCITY);
        double shooterVelocity = shooterControl.velocity();
        shooterControl.update(targetVelocity);

        /* ---------- INTAKE ---------- */
        double intakePower = 0, transferPower = 0;
        if (gamepad2.cross) {
            // Spit everything out
            intakePower = INTAKE_REVERSE_POWER;
            transferPower = TRANSFER_REVERSE_POWER;
        } else if (gamepad2.left_bumper) {
            // Intake only
            intakePower = INTAKE_IN_POWER;
        } else if (gamepad2.right_bumper) {
            // Intake + transfer (power depends on the shooter mode, see BioBuzzConfig)
            intakePower = shooterControl.intakeTransferPower();
            transferPower = shooterControl.intakeTransferPower();
        }
        intake.setPower(intakePower);
        transfer.setPower(transferPower);

        /* ---------- TELEMETRY ---------- */
        boolean ready = Math.abs(targetVelocity - shooterVelocity) < SHOOTER_READY_TOLERANCE;

        if (gamepad2.dpad_down) {
            resetDipStats();
        }
        updateDipStats(nowNanos, targetVelocity, shooterVelocity);

        telemetry.addData("Loop time (ms)", "%.1f (%.0f Hz), max %.1f", loopMs, loopMs > 0 ? 1000.0 / loopMs : 0, maxLoopMs);
        telemetry.addData("Alliance / shooter", "%s / %s", alliance, shooterControl.name());
        telemetry.addData("Active goal", "%s%s", turret.isTargetingSecondaryGoal() ? "SECONDARY" : "PRIMARY",
                Double.isNaN(alliance.autoGoalSplitY) ? "" : "  (auto by y, split " + alliance.autoGoalSplitY + ")");
        telemetry.addData("Holding", activeHold);
        telemetry.addData("--- POSE ---", "");
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Speed (in/s)", "%.1f", Math.hypot(velX, velY));
        telemetry.addData("Distance to goal (in)", "%.2f", distance);
        telemetry.addData("--- TURRET (options = re-zero, dpad L/R = trim) ---", "");
        telemetry.addData("Angle target / actual (deg)", "%.1f / %.1f", turret.getTargetAngle(), turret.getAngle());
        telemetry.addData("Angle error (deg)", "%.1f", turret.getTargetAngle() - turret.getAngle());
        telemetry.addData("Trim (deg)", "%.1f", turret.getTrim());
        telemetry.addData("Raw ticks", turret.getRawTicks());
        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target (ticks/s)", "%.0f", targetVelocity);
        telemetry.addData("Actual (ticks/s)", "%.0f", shooterVelocity);
        telemetry.addData("Velocity error", "%.0f", targetVelocity - shooterVelocity);
        shooterControl.addTelemetry(telemetry);
        telemetry.addData("Ready", ready ? "YES" : "no");
        telemetry.addData("--- SHOT TEST (gamepad2 dpad down = reset) ---", "");
        telemetry.addData("Tracking", dipArmed ? "ON" : "waiting for READY");
        telemetry.addData("MAX DIP (ticks/s below target)", "%.0f", maxDip);
        telemetry.addData("Lowest speed", lowestSpeed == Double.MAX_VALUE ? "-" : String.format("%.0f", lowestSpeed));
        telemetry.addData("Recovery time (last / worst)", "%s / %s%s",
                seconds(lastRecoverySec), seconds(worstRecoverySec), recovering ? "  (recovering now...)" : "");
        telemetry.addData("SEND TO CLAUDE", "%s %s maxDip=%.0f recovery=%s",
                shooterControl.name(), shooterControl.tuningSummary(), maxDip, seconds(worstRecoverySec));
        telemetry.addData("--- FEED ---", "");
        telemetry.addData("Intake / Transfer power", "%.2f / %.2f", intakePower, transferPower);
        telemetry.addData("Gamepad2 assigned", gamepad2.getGamepadId() != Gamepad.ID_UNASSOCIATED);
        telemetry.addData("Blocker", gamepad2.triangle ? "OPEN" : "BLOCKING");
        telemetry.addData("Flower1", flowerActive ? "ACTIVE" : "REST");
        telemetry.update();
    }

    /** Shooter target (ticks/s) while USE_FIXED_SHOOTER_VELOCITY is on. The experiment teleops override this. */
    protected double fixedShooterVelocity() {
        return SHOOTER_FIXED_VELOCITY + shooterControl.velocityOffset();
    }

    /** Forget the dip stats. They also start over until the flywheel has been ready once. */
    private void resetDipStats() {
        dipArmed = false;
        maxDip = 0;
        lowestSpeed = Double.MAX_VALUE;
        recovering = false;
        lastRecoverySec = -1;
        worstRecoverySec = -1;
    }

    /**
     * Tracks how far the flywheel drops below its target while shooting (blocker held open) and how
     * long it takes to come back inside SHOOTER_READY_TOLERANCE. Starts counting once the wheel has
     * been ready, and counts until reset (dpad down), so reset right before a test. The idle speed
     * ripple (a few tens of ticks/s) is the baseline; a real shot dips past it.
     */
    private void updateDipStats(long nowNanos, double target, double velocity) {
        double error = target - velocity;
        if (!dipArmed) {
            dipArmed = Math.abs(error) <= SHOOTER_READY_TOLERANCE;
            return;
        }

        maxDip = Math.max(maxDip, error);
        lowestSpeed = Math.min(lowestSpeed, velocity);

        if (!recovering && error > SHOOTER_READY_TOLERANCE) {
            recovering = true;
            dipStartNs = nowNanos;
        } else if (recovering && error <= SHOOTER_READY_TOLERANCE) {
            recovering = false;
            lastRecoverySec = (nowNanos - dipStartNs) / 1e9;
            worstRecoverySec = Math.max(worstRecoverySec, lastRecoverySec);
        }
    }

    private static String seconds(double s) {
        return s < 0 ? "-" : String.format("%.2f s", s);
    }

    /**
     * Hold a button to go to and hold its point; let go to drive again. Same rules as Decode:
     * only one hold at a time, and a hold in progress is not interrupted by another button.
     */
    private void updateHold() {
        if (activeHold != HoldTarget.NONE && !isPressed(activeHold)) {
            activeHold = HoldTarget.NONE;
        }
        if (activeHold != HoldTarget.NONE) return;

        for (HoldTarget target : new HoldTarget[]{
                HoldTarget.EMPTY_GATE, HoldTarget.PARK, HoldTarget.SHOOT_FAR, HoldTarget.SHOOT_CLOSE}) {
            if (isPressed(target)) {
                follower.hold(poseFor(target));
                activeHold = target;
                return;
            }
        }
    }

    private boolean isPressed(HoldTarget target) {
        switch (target) {
            case EMPTY_GATE:  return gamepad1.square;
            case PARK:        return gamepad1.circle;
            // On RED the two shoot buttons are swapped (right bumper = far, left bumper = close).
            case SHOOT_FAR:   return alliance == BioBuzzAlliance.RED ? gamepad1.right_bumper : gamepad1.left_bumper;
            case SHOOT_CLOSE: return alliance == BioBuzzAlliance.RED ? gamepad1.left_bumper : gamepad1.right_bumper;
            default:          return false;
        }
    }

    private Pose poseFor(HoldTarget target) {
        switch (target) {
            case EMPTY_GATE:  return EMPTY_GATE_POSE;
            case PARK:        return alliance.parkPose;
            case SHOOT_FAR:   return alliance.shootFarPose;
            default:          return alliance.shootClosePose;
        }
    }

    @Override
    public void stop() {
        // turret.stop();
        shooterControl.stop();
        intake.setPower(0);
        transfer.setPower(0);
    }
}
