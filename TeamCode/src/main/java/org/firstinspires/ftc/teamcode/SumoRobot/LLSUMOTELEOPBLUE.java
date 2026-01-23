package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class LLSUMOTELEOPBLUE extends OpMode {

    /* ===================== HARDWARE ===================== */
    private DcMotor fr, fl, br, bl;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    private Follower follower;
    private Limelight3A limelight;

    /* ===================== FIELD POSES ===================== */
    private final Pose startPose   = new Pose(54, 8, Math.toRadians(90));
    private final Pose parkPose    = new Pose(104.67, 33, Math.toRadians(0));
    private final Pose shootFar    = new Pose(66, 18, Math.toRadians(118));
    private final Pose shootClose  = new Pose(62, 108, Math.toRadians(145));
    private final Pose emptyGate   = new Pose(14.7, 70.5, Math.toRadians(270));

    /* ===================== STATE ===================== */
    private boolean holdingEmptyGate, holdingPark, holdingShootFar, holdingShootClose;

    /* ===================== SHOOTER ===================== */
    private static final double LOW_VELOCITY  = 1500;
    private static final double HIGH_VELOCITY = 1875;

    /* ===================== KICKER ===================== */
    private static final double KICKER_OUT = 0.52;
    private static final double KICKER_IN  = 0.35;
    private static final long   KICK_TIME  = 130;
    private boolean kicking = false;
    private long kickStartTime = 0;
    private boolean kickerLast = false;

    /* ===================== LIMELIGHT ===================== */
    private static final long RELOCALIZE_COOLDOWN = 500;
    private static final int  SAMPLE_COUNT = 5;
    private static final double FIELD_HALF = 72.0; // FTC field is 144x144

    private Pose[] poseBuffer = new Pose[SAMPLE_COUNT];
    private int poseIndex = 0, poseCount = 0;
    private long lastRelocalize = 0;
    private boolean wasMoving = false;

    /* ===================== INIT ===================== */
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1.0);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        PIDFCoefficients pidf = new PIDFCoefficients(100, 0, 0, 15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_IN);
    }

    /* ===================== START ===================== */
    @Override
    public void start() {
        follower.setPose(startPose);
        follower.startTeleopDrive();
    }

    /* ===================== LOOP ===================== */
    @Override
    public void loop() {
        follower.update();

        // 🔑 REQUIRED for MegaTag2
        double yawDeg = Math.toDegrees(follower.getPose().getHeading());
        limelight.updateRobotOrientation(yawDeg);

        relocalizeWithAprilTags();

        /* -------- Hold Point Controls -------- */
        handleHold(gamepad1.square, emptyGate, () -> holdingEmptyGate, v -> holdingEmptyGate = v);
        handleHold(gamepad1.circle, parkPose, () -> holdingPark, v -> holdingPark = v);
        handleHold(gamepad1.left_bumper, shootFar, () -> holdingShootFar, v -> holdingShootFar = v);
        handleHold(gamepad1.right_bumper, shootClose, () -> holdingShootClose, v -> holdingShootClose = v);

        boolean holding = holdingEmptyGate || holdingPark || holdingShootFar || holdingShootClose;

        follower.setTeleOpDrive(
                holding ? 0 : -gamepad1.left_stick_y,
                holding ? 0 : -gamepad1.left_stick_x,
                holding ? 0 : -gamepad1.right_stick_x,
                true
        );

        /* -------- Kicker -------- */
        boolean kickNow = gamepad2.triangle;
        if (kickNow && !kickerLast && !kicking) {
            kicking = true;
            kickStartTime = System.currentTimeMillis();
            kicker.setPosition(KICKER_OUT);
        }
        if (kicking && System.currentTimeMillis() - kickStartTime >= KICK_TIME) {
            kicker.setPosition(KICKER_IN);
            kicking = false;
        }
        kickerLast = kickNow;

        /* -------- Shooter / Intake -------- */
        double velocity = gamepad2.right_bumper ? HIGH_VELOCITY :
                gamepad2.left_bumper  ? LOW_VELOCITY  : 0;

        shooter1.setVelocity(velocity);
        shooter2.setVelocity(velocity);
        intake.setPower(velocity > 0 ? 1.0 : 0.0);

        telemetry.addData("Pedro Pose", follower.getPose());
        telemetry.update();
    }

    /* ===================== APRILTAG RELOCALIZATION ===================== */
    private void relocalizeWithAprilTags() {
        boolean moving =
                Math.abs(gamepad1.left_stick_x) > 0.05 ||
                        Math.abs(gamepad1.left_stick_y) > 0.05 ||
                        Math.abs(gamepad1.right_stick_x) > 0.05;

        if (moving) {
            poseCount = 0;
            wasMoving = true;
            return;
        }

        if (wasMoving) {
            poseCount = 0;
            wasMoving = false;
            return;
        }

        if (System.currentTimeMillis() - lastRelocalize < RELOCALIZE_COOLDOWN) return;

        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return;

        Pose3D p = r.getBotpose_MT2();
        if (p == null) return;

        double llX = p.getPosition().x;
        double llY = p.getPosition().y;
        double llHeadingDeg = p.getOrientation().getYaw();

        // ❌ Missing field layout → (0,0,0)
        if (llX == 0 && llY == 0 && llHeadingDeg == 0) return;

        // ✅ CENTER → BOTTOM-LEFT TRANSFORM
        double pedroX = llX + FIELD_HALF;
        double pedroY = llY + FIELD_HALF;
        double pedroHeading = Math.toRadians(llHeadingDeg);

        Pose pose = new Pose(pedroX, pedroY, pedroHeading);

        poseBuffer[poseIndex++] = pose;
        poseIndex %= SAMPLE_COUNT;
        poseCount++;

        if (poseCount >= SAMPLE_COUNT) {
            double x = 0, y = 0, sin = 0, cos = 0;
            for (Pose ps : poseBuffer) {
                x += ps.getX();
                y += ps.getY();
                sin += Math.sin(ps.getHeading());
                cos += Math.cos(ps.getHeading());
            }
            follower.setPose(new Pose(
                    x / SAMPLE_COUNT,
                    y / SAMPLE_COUNT,
                    Math.atan2(sin / SAMPLE_COUNT, cos / SAMPLE_COUNT)
            ));
            lastRelocalize = System.currentTimeMillis();
        }
    }

    /* ===================== UTIL ===================== */
    private void handleHold(boolean pressed, Pose pose,
                            java.util.function.BooleanSupplier get,
                            java.util.function.Consumer<Boolean> set) {
        if (pressed && !get.getAsBoolean()) {
            follower.holdPoint(pose);
            set.accept(true);
        }
        if (!pressed && get.getAsBoolean()) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            set.accept(false);
        }
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
