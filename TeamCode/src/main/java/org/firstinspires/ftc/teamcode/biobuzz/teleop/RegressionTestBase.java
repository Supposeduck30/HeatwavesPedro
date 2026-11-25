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
 * Shooter regression data collection (distance -> flywheel velocity, for a fit in Desmos).
 * Use {@link RegressionTestRed} or {@link RegressionTestBlue}.
 *
 * START: always the teleop's default start pose for the alliance (BioBuzzAlliance.startPose, Red = (54.6, 7.4) facing
 * up the field), never the pose from an auto. Place the robot there, turret straight forward, then press init.
 *
 * The turret auto-aims at the same goal point the teleop uses (picked by the robot's y, split at 70), and the
 * distance shown is from the turret to that goal point, the same number the teleop will feed the regression.
 *
 * HOW TO COLLECT: drive to a spot, adjust the velocity until shots go in, press gamepad2 square to save
 * (distance, velocity). Repeat at different distances. The "DESMOS" lines at the bottom are ready to type in:
 *     D = [ ... ]      V = [ ... ]      then in Desmos:   V ~ a D^2 + b D + c
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose, cross = toggle flower
 * gamepad2: dpad up / down    = velocity + / - BIG_STEP   (default 50)
 *           dpad right / left = velocity + / - SMALL_STEP (default 10)
 *           right trigger (hold) = shoot, right bumper = intake + transfer, left bumper = intake only,
 *           cross = reverse both, triangle (hold) = open blocker, options = turret straight forward (re-zero),
 *           square = SAVE a sample, share = clear the samples
 * Panels (Configurables -> "RegressionTestBase"): TARGET_VELOCITY, BIG_STEP, SMALL_STEP.
 * Panels graph keys: "Target", "Actual", "Error", "Distance".
 */
@Configurable
public abstract class RegressionTestBase extends OpMode {

    // ================= EDITABLE IN PANELS =================
    /** Flywheel target (ticks/s). Starts at SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET. */
    public static double TARGET_VELOCITY = SHOOTER_FIXED_VELOCITY + PIDF_VELOCITY_OFFSET;
    /** gamepad2 dpad up / down step (ticks/s). */
    public static double BIG_STEP = 50;
    /** gamepad2 dpad right / left step (ticks/s). */
    public static double SMALL_STEP = 10;
    // ======================================================

    private static final int MAX_SAMPLES = 20;

    private final BioBuzzAlliance alliance;

    private Follower follower;
    private DcMotorEx shooter;
    private DcMotor intake, transfer;
    private Servo blocker, flower;
    private BioBuzzTurret turret;
    private TelemetryManager panels;

    private boolean flowerActive = false;
    private boolean lastG1Cross, lUp, lDown, lLeft, lRight, lSquare, lShare;
    private boolean firstLoop = true;

    private final List<double[]> samples = new ArrayList<>(); // {distance, velocity}

    protected RegressionTestBase(BioBuzzAlliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(alliance.startPose); // always the teleop default start pose, never the auto's

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

        turret = new BioBuzzTurret(hardwareMap, alliance); // encoder reset here = straight forward

        panels = PanelsTelemetry.INSTANCE.getTelemetry();
        telemetry.addData("Status", "Initialized (" + alliance + " regression)");
        telemetry.addData("Start pose", "%.1f, %.1f, %.0f deg  (place the robot here, turret forward)",
                alliance.startPose.x(), alliance.startPose.y(), Math.toDegrees(alliance.startPose.heading()));
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
        Pose pose = follower.pose();

        /* ---------- TARGET VELOCITY: dpad up/down = big step, right/left = small step ---------- */
        if (gamepad2.dpad_up && !lUp)       TARGET_VELOCITY += BIG_STEP;
        if (gamepad2.dpad_down && !lDown)   TARGET_VELOCITY -= BIG_STEP;
        if (gamepad2.dpad_right && !lRight) TARGET_VELOCITY += SMALL_STEP;
        if (gamepad2.dpad_left && !lLeft)   TARGET_VELOCITY -= SMALL_STEP;
        lUp = gamepad2.dpad_up; lDown = gamepad2.dpad_down;
        lRight = gamepad2.dpad_right; lLeft = gamepad2.dpad_left;
        TARGET_VELOCITY = Math.max(0, Math.min(MAX_SHOOTER_VELOCITY, TARGET_VELOCITY));

        /* ---------- TURRET: same goal choice as the teleop ---------- */
        if (!Double.isNaN(alliance.autoGoalSplitY)) {
            turret.setSecondaryGoal(alliance.useSecondaryGoalAt(pose.y()));
        }
        if (gamepad2.options) {
            turret.resetEncoder();
        }
        turret.aimAtGoal(pose, 0, 0); // no shot leading (same as teleop)
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
        if (gamepad2.square && !lSquare) {
            samples.add(new double[]{distance, TARGET_VELOCITY});
            if (samples.size() > MAX_SAMPLES) samples.remove(0);
        }
        if (gamepad2.share && !lShare) samples.clear();
        lSquare = gamepad2.square;
        lShare = gamepad2.share;

        /* ---------- TELEMETRY ---------- */
        boolean ready = Math.abs(TARGET_VELOCITY - velocity) < SHOOTER_READY_TOLERANCE;
        telemetry.addData("DISTANCE TO GOAL (in)", "%.2f", distance);
        telemetry.addData("VELOCITY target / actual", "%.0f / %.0f  %s", TARGET_VELOCITY, velocity,
                ready ? "READY" : "recovering");
        telemetry.addData("Steps", "dpad up/down +-%.0f, right/left +-%.0f", BIG_STEP, SMALL_STEP);
        telemetry.addData("Goal", turret.isTargetingSecondaryGoal() ? "SECONDARY" : "PRIMARY");
        telemetry.addData("X / Y / heading", "%.1f / %.1f / %.1f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Turret angle / target", "%.1f / %.1f", turret.getAngle(), turret.getTargetAngle());
        telemetry.addData("Samples (square = save, share = clear)", samples.size());
        for (int i = 0; i < samples.size(); i++) {
            telemetry.addData("  " + (i + 1), "%.1f in -> %.0f", samples.get(i)[0], samples.get(i)[1]);
        }
        if (!samples.isEmpty()) {
            telemetry.addData("DESMOS D", desmosList(0));
            telemetry.addData("DESMOS V", desmosList(1));
        }
        telemetry.update();

        panels.addData("Target", TARGET_VELOCITY);
        panels.addData("Actual", velocity);
        panels.addData("Error", TARGET_VELOCITY - velocity);
        panels.addData("Distance", distance);
        panels.update();
    }

    /** "[a, b, c]" of one column of the samples, for pasting into Desmos. */
    private String desmosList(int column) {
        StringBuilder sb = new StringBuilder("[");
        for (int i = 0; i < samples.size(); i++) {
            if (i > 0) sb.append(", ");
            sb.append(String.format(column == 0 ? "%.1f" : "%.0f", samples.get(i)[column]));
        }
        return sb.append("]").toString();
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
