package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Drivetrain + turret ONLY (no shooter, intake, transfer or blocker; the flower servo is just held up), for coding the turret.
 * Same start pose, goals, relocalize and turret logic as {@link BioBuzzTeleOpV2Base}; the turret is
 * {@link BioBuzzTurret}. Tunables: {@link BioBuzzConfig} (TURRET_*) and {@link BioBuzzAlliance} (goals, poses).
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose, dpad up = relocalize
 * gamepad2: circle = toggle primary/secondary goal (ignored when the alliance auto-picks by y),
 *           dpad left / right = turret aim trim (left = counter-clockwise),
 *           (DISABLED) options = re-home the turret on the magnet (spins CLOCKWISE into the left edge of the magnet band)
 *           share   = fallback: declare the turret's current position to be straight forward (90 deg)
 *
 * LIMIT SWITCH DISABLED: homing / re-homing is commented out in BioBuzzTurret.
 * Start the turret pointing straight forward at init. Right after start it homes on the magnet (spins clockwise until
 * it hits it, then aims). After that, every clockwise pass over the magnet re-homes it, which undoes belt/gear skips.
 */
public abstract class TurretDriveTest extends OpMode {

    private final BioBuzzAlliance alliance;

    private Follower follower;
    private BioBuzzTurret turret;
    private Servo flower;

    private boolean lastGamepad2Circle = false;
    private boolean lastGamepad2Options = false;

    // Field-frame velocity from pose differences (used for shot leading)
    private Pose lastPose = null;
    private long lastNanos = 0;
    private double velX = 0, velY = 0;

    private boolean firstLoop = true;
    private long lastLoopNanos = 0;
    private double loopMs = 0;

    protected TurretDriveTest(BioBuzzAlliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        // Always start from the alliance start pose (Red: 54.5, 7.4, heading 90). A pose left over from an earlier
        // auto is ignored here so a stale heading can never point the turret the wrong way.
        follower.setPose(alliance.startPose);

        // Keep the flower servo powered and up (FLOWER_REST) from init on; an unpowered servo falls.
        flower = hardwareMap.get(Servo.class, FLOWER);
        flower.setPosition(FLOWER_REST);

        // Always assume the turret starts at TURRET_START_ANGLE_DEG (90 = straight forward): the encoder is zeroed
        // here and any turret position left by auto is ignored.
        turret = new BioBuzzTurret(hardwareMap, alliance);
        telemetry.addData("Turret", "starts at %.0f deg (point it straight forward at init)", TURRET_START_ANGLE_DEG);
        telemetry.addData("Status", "Initialized (" + alliance + ", turret + drivetrain only)");
        telemetry.update();
    }

    @Override
    public void start() {
        // Motors can't move during init, so home as soon as the opmode starts.
//        turret.startHoming();
    }

    @Override
    public void loop() {
        flower.setPosition(FLOWER_REST); // hold it up every loop
        follower.update();

        // The first update just lets the localizer settle.
        if (firstLoop) {
            firstLoop = false;
            return;
        }

        long nowNanos = System.nanoTime();
        if (lastLoopNanos != 0) {
            loopMs = (nowNanos - lastLoopNanos) / 1e6;
        }
        lastLoopNanos = nowNanos;

        if (gamepad1.triangle) {
            follower.setPose(alliance.resetPose);
        }
        // Relocalize: drive to the marked spot, press dpad up, and Pedro restarts odometry from there.
        if (gamepad1.dpad_up) {
            follower.setPose(alliance.relocalizePose);
        }

        /* ---------- DRIVE ---------- */
        follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

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
            // Automatic: per-alliance rule, see BioBuzzAlliance.useSecondaryGoalAt (circle is ignored).
            turret.setSecondaryGoal(alliance.useSecondaryGoalAt(pose.y()));
        } else if (circleNow && !lastGamepad2Circle) {
            turret.toggleGoal();
        }
        lastGamepad2Circle = circleNow;

        /* ---------- TURRET ---------- */
//        boolean optionsNow = gamepad2.options;
//        if (optionsNow && !lastGamepad2Options) {
//            turret.startHoming();
//        }
//        lastGamepad2Options = optionsNow;
        if (gamepad2.share) {
            turret.resetEncoder(); // fallback if the switch isn't working; also cancels homing
        }
        double trimRate = (gamepad2.dpad_left ? TURRET_TRIM_RATE_DEG_PER_SEC : 0)
                - (gamepad2.dpad_right ? TURRET_TRIM_RATE_DEG_PER_SEC : 0);
        turret.adjustTrim(trimRate, loopMs / 1000.0);
        turret.aimAtGoal(pose, velX, velY); // runs homing instead of aiming while homing is active

        /* ---------- TELEMETRY ---------- */
        telemetry.addData("Alliance", alliance);
        telemetry.addData("Active goal", "%s%s", turret.isTargetingSecondaryGoal() ? "SECONDARY" : "PRIMARY",
                Double.isNaN(alliance.autoGoalSplitY) ? "" : "  (auto by y, split " + alliance.autoGoalSplitY + ")");
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.addData("Speed (in/s)", "%.1f", Math.hypot(velX, velY));
        telemetry.addData("Distance to goal (in)", "%.2f", turret.getDistanceToGoal(pose));
        boolean secondary = turret.isTargetingSecondaryGoal();
        double goalX = secondary ? alliance.secondaryGoalX : alliance.primaryGoalX;
        double goalY = secondary ? alliance.secondaryGoalY : alliance.primaryGoalY;
        double pivotX = pose.x() + TURRET_OFFSET_FORWARD * Math.cos(pose.heading()) - TURRET_OFFSET_LEFT * Math.sin(pose.heading());
        double pivotY = pose.y() + TURRET_OFFSET_FORWARD * Math.sin(pose.heading()) + TURRET_OFFSET_LEFT * Math.cos(pose.heading());
        double fieldAngle = Math.toDegrees(Math.atan2(goalY - pivotY, goalX - pivotX));
        telemetry.addData("--- DEBUG (expected at Red start: field ~87, front-relative ~-3, target ~87) ---", "");
        telemetry.addData("Goal x / y", "%.2f / %.2f", goalX, goalY);
        telemetry.addData("Pivot x / y", "%.2f / %.2f", pivotX, pivotY);
        telemetry.addData("Field angle to goal (deg)", "%.1f", fieldAngle);
        telemetry.addData("Goal rel. to robot front (deg)", "%.1f", fieldAngle - Math.toDegrees(pose.heading()));
        telemetry.addData("--- TURRET (options = re-home, share = re-zero to 90, dpad L/R = trim) ---", "");
//        telemetry.addData("Homing", "%s: %s", turret.getHomeState(), turret.getHomeMessage());
//        telemetry.addData("Magnet", turret.isLimitPressed() ? "DETECTED" : "no");
//        telemetry.addData("Re-homes / last correction", "%d / %.1f deg", turret.getRehomeCount(), turret.getLastRehomeCorrection());
        telemetry.addData("Angle target / actual (deg)", "%.1f / %.1f", turret.getTargetAngle(), turret.getAngle());
        telemetry.addData("Angle error (deg)", "%.1f", turret.getTargetAngle() - turret.getAngle());
        telemetry.addData("Trim (deg)", "%.1f", turret.getTrim());
        telemetry.addData("Raw ticks", turret.getRawTicks());
        telemetry.addData("Loop (ms)", "%.1f", loopMs);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }

    @TeleOp(name = "Turret + Drive Blue", group = "BioBuzz")
    public static class Blue extends TurretDriveTest {
        public Blue() { super(BioBuzzAlliance.BLUE); }
    }

    @TeleOp(name = "Turret + Drive Red", group = "BioBuzz")
    public static class Red extends TurretDriveTest {
        public Red() { super(BioBuzzAlliance.RED); }
    }
}
