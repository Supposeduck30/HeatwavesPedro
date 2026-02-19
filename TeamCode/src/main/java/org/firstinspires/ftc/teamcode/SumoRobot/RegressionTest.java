package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class RegressionTest extends OpMode {

    private DcMotor fr, fl, br, bl;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    private Follower follower;
    private Limelight3A limelight;

    // TURRET CONTROLLER
    private TurretController turretController;

    private Servo rgbIndicator;
    private static final double RED = 0.277;
    private static final double BLUE = 0.5;

    private final Pose parkPose  = new Pose(39.33, 33, Math.toRadians(180));
    private final Pose shootFar  = new Pose(78, 18, Math.toRadians(64));
    private final Pose resetPose = new Pose(7, 9, Math.toRadians(90));
    private final Pose shootClose = new Pose(82, 108, Math.toRadians(30.5));
    private final Pose emptyGate = new Pose(132.5, 68.5, Math.toRadians(180));

    private boolean kicking = false;
    private long kickStartTime = 0;
    private boolean kickerButtonLast = false;
    private static final long KICK_TIME = 129;

    private boolean holdingEmptyGate = false;
    private boolean holdingPark = false;
    private boolean holdingShootFar = false;
    private boolean holdingShootClose = false;
    private boolean aligningToTag = false;

    public static final double KICKER_OUT = 0.58;
    public static final double KICKER_IN  = 0.36;

    // SINGLE CONFIGURABLE SHOOTER VELOCITY
    // Adjust this value in the configuration panel
    public static double SHOOTER_VELOCITY = 900; // Default value

    private static final double ALIGN_KP = 0.015;
    private static final double ALIGN_MAX_POWER = 0.45;
    private static final double ALIGN_TOLERANCE = 1.0;
    private static final int TARGET_TAG_ID = 24;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);
        Pose startPose = new Pose(54.5, 8, Math.toRadians(90));
        follower.setStartingPose(startPose);

        rgbIndicator = hardwareMap.get(Servo.class, "RGB");
        rgbIndicator.setPosition(RED);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.start();

        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        intake = hardwareMap.get(DcMotor.class, "Intake");
        kicker = hardwareMap.get(Servo.class, "Kicker");

        // INITIALIZE TURRET
        turretController = new TurretController(hardwareMap, "Turret");
        turretController.resetEncoder(); // starts at 0° (right)

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

        PIDFCoefficients pidf = new PIDFCoefficients(80, 0, 0, 15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_IN);

        // Read configuration values
        SHOOTER_VELOCITY = getShooterVelocityFromConfig();

        telemetry.addData("Status", "Initialized with Turret");
        telemetry.addData("Shooter Velocity", "%.0f (configurable)", SHOOTER_VELOCITY);
        telemetry.addData("Controls", "RB: Shoot + Intake | LB: Intake Only | X: Reverse Intake");
        telemetry.update();
    }

    private boolean firstLoop = true;

    // For velocity calculation
    private Pose lastPose = null;
    private long lastTime = 0;

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // Skip turret aiming on first loop
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
        boolean xButtonNow     = gamepad1.cross;

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

        if (xButtonNow && !aligningToTag) aligningToTag = true;
        if (!xButtonNow && aligningToTag) aligningToTag = false;

        boolean holdingPoint = holdingEmptyGate || holdingPark || holdingShootFar || holdingShootClose;

        double driveForward = (holdingPoint || aligningToTag) ? 0 : -gamepad1.left_stick_y;
        double driveStrafe  = (holdingPoint || aligningToTag) ? 0 : -gamepad1.left_stick_x;
        double driveTurn    = 0;

        if (aligningToTag) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {
                        double tx = fiducial.getTargetXDegrees();
                        if (Math.abs(tx) > ALIGN_TOLERANCE) {
                            driveTurn = -tx * ALIGN_KP;
                            driveTurn = Math.max(-ALIGN_MAX_POWER, Math.min(ALIGN_MAX_POWER, driveTurn));
                        }
                        break;
                    }
                }
            }
        } else {
            driveTurn = holdingPoint ? 0 : -gamepad1.right_stick_x;
        }

        follower.setTeleOpDrive(driveForward, driveStrafe, driveTurn, true);

        /* ---------- TURRET CONTROL WITH PREDICTIVE AIMING ---------- */
        Pose currentPose = follower.getPose();

        // Calculate velocity manually
        Pose velocity;
        if (lastPose == null) {
            // First loop - no velocity yet
            velocity = new Pose(0, 0, 0);
        } else {
            long currentTime = System.currentTimeMillis();
            double dt = (currentTime - lastTime) / 1000.0; // Convert to seconds

            if (dt > 0) {
                double velX = (currentPose.getX() - lastPose.getX()) / dt;
                double velY = (currentPose.getY() - lastPose.getY()) / dt;
                double velHeading = (currentPose.getHeading() - lastPose.getHeading()) / dt;
                velocity = new Pose(velX, velY, velHeading);
            } else {
                velocity = new Pose(0, 0, 0);
            }
        }

        // Update for next loop
        lastPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        lastTime = System.currentTimeMillis();

        // Use predictive aiming
        turretController.aimAtGoalWithPrediction(currentPose, velocity);

        if (gamepad2.options) {
            turretController.resetEncoder();
        }

        /* ---------- KICKER ---------- */
        boolean kickerButtonNow = gamepad2.triangle;
        if (kickerButtonNow && !kickerButtonLast && !kicking) {
            kicking = true;
            kickStartTime = System.currentTimeMillis();
            kicker.setPosition(KICKER_OUT);
        }
        if (kicking && System.currentTimeMillis() - kickStartTime >= KICK_TIME) {
            kicker.setPosition(KICKER_IN);
            kicking = false;
        }
        kickerButtonLast = kickerButtonNow;

        /* ---------- SIMPLIFIED SHOOTER + INTAKE ---------- */
        double targetVelocity = 0;
        double intakePower = 0;

        if (gamepad2.cross) {               // X → intake reverse
            intakePower = -0.4;
        }
        else if (gamepad2.left_bumper) {    // LB → intake only
            intakePower = 0.69;
        }
        else if (gamepad2.right_bumper) {   // RB → shooter + intake + auto aim turret
            targetVelocity = SHOOTER_VELOCITY;  // Use the single configurable velocity
            intakePower = 1.0;
        }

        shooter1.setVelocity(targetVelocity);
        shooter2.setVelocity(targetVelocity);
        intake.setPower(intakePower);

        /* ---------- FIXED TELEMETRY ---------- */
        double targetAngle = turretController.calculateTurretAngle(currentPose);
        double currentAngle = turretController.getCurrentAngle();

        /* ---------- TURRET DEBUG TELEMETRY ---------- */
        //int rawTicks = turretController.getRawTicks();
       // double rawAngle = turretController.getRawAngle();

        telemetry.addData("--- SHOOTER CONFIG ---", "");
        telemetry.addData("Current Velocity", "%.0f", targetVelocity);
        telemetry.addData("Configurable Value", "%.0f (adjust in panel)", SHOOTER_VELOCITY);

        telemetry.addData("--- TURRET ---", "");
        telemetry.addData("Target Angle (0=right, 180=left)", "%.2f°", targetAngle);
        telemetry.addData("Current Angle", "%.2f°", currentAngle);
        telemetry.addData("Distance to Goal", "%.2f in", turretController.getDistanceToGoal(currentPose));
        //telemetry.addData("Goal Reachable", turretController.isGoalReachable(currentPose));

        telemetry.addData("--- TURRET DEBUG ---", "");
       // telemetry.addData("Raw Encoder Ticks", rawTicks);
        //telemetry.addData("Raw Angle (deg)", "%.2f", rawAngle);
       // telemetry.addData("Motor Power", "%.2f", turretController.getMotorPower());
        //telemetry.addData("Is Busy", turretController.isBusy());
       // telemetry.addData("Motor Mode", turretController.isBusy() ? "MOVING" : "STOPPED");

        telemetry.update();
    }

    /**
     * Read shooter velocity from configuration
     * This value can be adjusted in the FTC Dashboard or configuration panel
     */
    private double getShooterVelocityFromConfig() {
        // Try to read from preferences or use default
        try {
            // You can implement reading from a configuration file here
            // For now, return the default value
            return SHOOTER_VELOCITY;
        } catch (Exception e) {
            return 1400; // Default fallback
        }
    }
}