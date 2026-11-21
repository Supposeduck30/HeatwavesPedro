package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import static org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Fresh BioBuzz TeleOp. Use {@link BioBuzzTeleOpV2Blue} or {@link BioBuzzTeleOpV2Red}.
 *
 * gamepad1: left stick = drive/strafe, right stick = turn, triangle = reset pose, cross = toggle flower
 * gamepad2: the flywheel ALWAYS runs (hub PIDF at SHOOTER_FIXED_VELOCITY) from the moment the OpMode starts,
 *           right trigger (hold) = SHOOT: opens the blocker and runs intake + transfer at PIDF_INTAKE_TRANSFER_POWER,
 *           right bumper = intake + transfer (blocker stays closed),
 *           left bumper = intake only, cross = reverse both, triangle (hold) = open blocker,
 *           circle = toggle primary/secondary goal (ignored when the alliance auto-picks by y),
 *           dpad left/right = turret trim (tap = one step, hold = continuous),
 *           dpad up = LOCK toggle: freeze the turret where it is (stops auto-aim); tap again to resume aiming,
 *           dpad down = zero turret (current position = straight forward)
 * The turret auto-aims at the goal every loop (unless locked).
 *
 * Shooter uses the hub velocity PIDF (SHOOTER_P / I / D / F) at SHOOTER_FIXED_VELOCITY. Tunables: {@link BioBuzzConfig}.
 */
public abstract class BioBuzzTeleOpV2Base extends OpMode {

    private final BioBuzzAlliance alliance;

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

        telemetry.addData("Status", "Initialized (" + alliance + ")");        telemetry.update();
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
        if (turretLocked) {
            turret.hold();
        } else {
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

    @Override
    public void stop() {
        turret.stop();
        shooter.setPower(0);
        intake.setPower(0);
        transfer.setPower(0);
    }
}
