package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class REDTELEOP extends OpMode {

    private DcMotor fr, fl, br, bl;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;
    private Follower follower;
    private Limelight3A limelight;

    // TURRET CONTROLLER
    private TurretControllerRED turretController;

    private Servo rgbIndicator;
    private static final double RED  = 0.277;
    private static final double BLUE = 0.5;

    private final Pose parkPose   = new Pose(39.33, 33, Math.toRadians(180));
    private final Pose shootFar   = new Pose(78, 18, Math.toRadians(64));
    private final Pose resetPose  = new Pose(7, 9, Math.toRadians(90));
    private final Pose shootClose = new Pose(82, 108, Math.toRadians(30.5));
    private final Pose emptyGate  = new Pose(132.5, 68.5, Math.toRadians(180));

    private boolean holdingEmptyGate  = false;
    private boolean holdingPark       = false;
    private boolean holdingShootFar   = false;
    private boolean holdingShootClose = false;
    private boolean aligningToTag     = false;

    // KICKER AS BLOCKER
    public static final double KICKER_BLOCK = 0.15;
    public static final double KICKER_OPEN  = 0.25;

    // Regression: velocity = m * distance + b
    //private static final double VELOCITY_SLOPE     = 6.58626;
    //private static final double VELOCITY_INTERCEPT = 1165.72046;

    private static final double VEL_A = 0.016966;
    private static final double VEL_B = 1.45242;
    private static final double VEL_C = 1235.32337;

    private static final double MAX_SHOOTER_VELOCITY = 2600;

    private static final double ALIGN_KP        = 0.015;
    private static final double ALIGN_MAX_POWER = 0.45;
    private static final double ALIGN_TOLERANCE = 1.0;
    private static final int    TARGET_TAG_ID   = 24;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        if (org.firstinspires.ftc.teamcode.SumoRobot.PedroPose.hasPoseFromAuto()){
            follower.setStartingPose(org.firstinspires.ftc.teamcode.SumoRobot.PedroPose.getTeleOpStartPose());
        } else {
            Pose startPose = new Pose(54.5,8,Math.toRadians(90));
            follower.setStartingPose(startPose);
        }

        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(1);
        //limelight.start();

        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        turretController = new TurretControllerRED(hardwareMap, "Turret");
        Integer autoTurretTicks = org.firstinspires.ftc.teamcode.SumoRobot.PedroPose.getTurretTicks();

        if (autoTurretTicks!=null) {
            turretController.setEncoderOffsetRED(autoTurretTicks);
            telemetry.addData("Turret mode", "Resumed from auto(" + autoTurretTicks + " ticks");
        } else {
            telemetry.addData("Turret mode", "Fresh start (0 ticks)");
        }

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        PIDFCoefficients pidf = new PIDFCoefficients(120, 0, 0, 25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_BLOCK);

        telemetry.addData("Status", "Initialized with Turret");
        telemetry.update();
    }

    private boolean firstLoop = true;

    private Pose lastPose = null;
    private long lastTime = 0;

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        if (firstLoop) {
            firstLoop = false;
            telemetry.addData("Status", "Initializing...");
            telemetry.update();
            return;
        }

        if (gamepad1.triangle) {
            follower.setPose(resetPose);
        }

        boolean leftBumperNow  = gamepad1.left_bumper;
        boolean rightBumperNow = gamepad1.right_bumper;
        boolean squareNow      = gamepad1.square;
        boolean circleNow      = gamepad1.circle;

        /* ---------- HOLD POINTS ---------- */
        if (squareNow && !holdingEmptyGate && !holdingPark && !holdingShootFar && !holdingShootClose && !aligningToTag) {
            follower.holdPoint(emptyGate); holdingEmptyGate = true;
        }
        if (!squareNow && holdingEmptyGate) { follower.breakFollowing(); follower.startTeleopDrive(); holdingEmptyGate = false; }

        if (circleNow && !holdingPark && !holdingEmptyGate && !holdingShootFar && !holdingShootClose && !aligningToTag) {
            follower.holdPoint(parkPose); holdingPark = true;
        }
        if (!circleNow && holdingPark) { follower.breakFollowing(); follower.startTeleopDrive(); holdingPark = false; }

        if (leftBumperNow && !holdingShootFar && !holdingEmptyGate && !holdingPark && !holdingShootClose && !aligningToTag) {
            follower.holdPoint(shootFar); holdingShootFar = true;
        }
        if (!leftBumperNow && holdingShootFar) { follower.breakFollowing(); follower.startTeleopDrive(); holdingShootFar = false; }

        if (rightBumperNow && !holdingShootClose && !holdingEmptyGate && !holdingPark && !holdingShootFar && !aligningToTag) {
            follower.holdPoint(shootClose); holdingShootClose = true;
        }
        if (!rightBumperNow && holdingShootClose) { follower.breakFollowing(); follower.startTeleopDrive(); holdingShootClose = false; }

        boolean holdingPoint = holdingEmptyGate || holdingPark || holdingShootFar || holdingShootClose;

        double driveForward = (holdingPoint || aligningToTag) ? 0 : -gamepad1.left_stick_y;
        double driveStrafe  = (holdingPoint || aligningToTag) ? 0 : -gamepad1.left_stick_x;
        double driveTurn    = holdingPoint ? 0 : -gamepad1.right_stick_x;

        follower.setTeleOpDrive(driveForward, driveStrafe, driveTurn, true);

        /* ---------- TURRET CONTROL WITH PREDICTIVE AIMING ---------- */
        Pose currentPose = follower.getPose();
        Pose velocity;

        long currentTime = System.currentTimeMillis();

        if (lastPose == null || lastTime == 0) {
            velocity = new Pose(0, 0, 0);
        } else {
            double dt = (currentTime - lastTime) / 1000.0;

            if (dt > 0.005) {
                double velX = (currentPose.getX() - lastPose.getX()) / dt;
                double velY = (currentPose.getY() - lastPose.getY()) / dt;

                double diffHeading = currentPose.getHeading() - lastPose.getHeading();
                while (diffHeading > Math.PI)  diffHeading -= 2 * Math.PI;
                while (diffHeading < -Math.PI) diffHeading += 2 * Math.PI;

                double velHeading = diffHeading / dt;
                velocity = new Pose(velX, velY, velHeading);
            } else {
                velocity = new Pose(0, 0, 0);
            }
        }

        lastPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        lastTime = currentTime;

        turretController.aimAtGoalWithPredictionRED(currentPose, velocity);

        if (gamepad2.options) {
            turretController.resetEncoderRED();
        }

        /* ---------- BLOCKER CONTROL ---------- */
        if (gamepad2.triangle) {
            kicker.setPosition(KICKER_OPEN);
        } else {
            kicker.setPosition(KICKER_BLOCK);
        }

        /* ---------- SHOOTER + INTAKE ---------- */
        // Target velocity is always distance-based — shooter runs continuously,
        // no idle speed switching. Intake and kicker control when balls actually fire.
        double distance = turretController.getDistanceToGoalRED(currentPose);
        double targetVelocity = (VEL_A*distance*distance)+(VEL_B*distance)+VEL_C;
        targetVelocity= Range.clip(targetVelocity,0,MAX_SHOOTER_VELOCITY);

        double intakePower = 0;
        if (gamepad2.cross) {               // X → intake reverse
            intakePower = -0.9;
        } else if (gamepad2.left_bumper) {  // LB → intake forward
            intakePower = 1.0;
        } else if (gamepad2.right_bumper) { // RB → intake forward (shoot)
            intakePower = 1.0;
        }

        shooter1.setVelocity(targetVelocity);
        shooter2.setVelocity(targetVelocity);
        intake.setPower(intakePower);

        /* ---------- TELEMETRY ---------- */
        double targetAngle  = turretController.calculateTurretAngleRED(currentPose);
        double currentAngle = turretController.getCurrentAngleRED();

        telemetry.addData("--- POSE ---", "");
        telemetry.addData("X Position", "%.2f", currentPose.getX());
        telemetry.addData("Y Position", "%.2f", currentPose.getY());
        telemetry.addData("Heading (degrees)", "%.2f", Math.toDegrees(currentPose.getHeading()));
        telemetry.addData("--- TURRET ---", "");
        telemetry.addData("Target Angle (0=right, 180=left)", "%.2f°", targetAngle);
        telemetry.addData("Current Angle", "%.2f°", currentAngle);
        telemetry.addData("Distance to Goal", "%.2f in", distance);
        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target Velocity", "%.0f ticks/s", targetVelocity);
        telemetry.addData("Shooter1 Actual", "%.0f ticks/s", shooter1.getVelocity());
        telemetry.addData("Shooter2 Actual", "%.0f ticks/s", shooter2.getVelocity());
        telemetry.addData("Velocity Error S1", "%.0f ticks/s", targetVelocity - shooter1.getVelocity());
        telemetry.addData("--- BLOCKER ---", "");
        telemetry.addData("Status", gamepad2.triangle ? "OPEN" : "BLOCKING");
        telemetry.addData("Motor direction of front right", fr.getDirection());
        telemetry.update();
    }
}