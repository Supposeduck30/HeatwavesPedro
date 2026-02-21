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

@TeleOp(name = "StateTeleOp", group = "TeleOp")
public class SumoTeleOpWithTurretThroughPut extends OpMode {

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

    private boolean holdingEmptyGate = false;
    private boolean holdingPark = false;
    private boolean holdingShootFar = false;
    private boolean holdingShootClose = false;
    private boolean aligningToTag = false;

    // KICKER AS BLOCKER
    public static final double KICKER_BLOCK = 0.15;  // Blocks balls (default)
    public static final double KICKER_OPEN = 0.25;   // Lets balls through (when triangle held)

    // Regression: velocity = m * distance + b
    private static final double VELOCITY_SLOPE = 6.58626;
    private static final double VELOCITY_INTERCEPT = 1165.72046;

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
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);


        PIDFCoefficients pidf = new PIDFCoefficients(400, 0, 0, 25);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Start with blocker blocking
        kicker.setPosition(KICKER_BLOCK);

        telemetry.addData("Status", "Initialized with Turret");
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
        Pose velocity;

        long currentTime = System.currentTimeMillis();

        if (lastPose == null || lastTime == 0) {
            velocity = new Pose(0, 0, 0);
        } else {
            double dt = (currentTime - lastTime) / 1000.0;

            // Prevent huge spikes if dt is tiny
            if (dt > 0.005) {
                double velX = (currentPose.getX() - lastPose.getX()) / dt;
                double velY = (currentPose.getY() - lastPose.getY()) / dt;

                // FIX HEADING WRAP AROUND
                // If heading jumps 359 -> 1, diff is -358. Real diff is +2.
                double diffHeading = currentPose.getHeading() - lastPose.getHeading();
                while (diffHeading > Math.PI) diffHeading -= 2 * Math.PI;
                while (diffHeading < -Math.PI) diffHeading += 2 * Math.PI;

                double velHeading = diffHeading / dt;

                velocity = new Pose(velX, velY, velHeading);
            } else {
                // If loop is too fast, keep previous velocity or 0
                velocity = new Pose(0,0,0);
            }
        }

        //lastPose = currentPose;  Objects are passed by reference, verify Pose is immutable or clone it
        // Safer to do:
        lastPose = new Pose(currentPose.getX(), currentPose.getY(), currentPose.getHeading());
        lastTime = currentTime;

        // Update Turret
        turretController.aimAtGoalWithPrediction(currentPose, velocity);

        if (gamepad2.options) {
            turretController.resetEncoder();
        }

        /* ---------- BLOCKER CONTROL (SIMPLE) ---------- */
        // Hold triangle to let balls through, release to block
        if (gamepad2.triangle) {
            kicker.setPosition(KICKER_OPEN);   // Let balls through
        } else {
            kicker.setPosition(KICKER_BLOCK);  // Block balls
        }

        /* ---------- SHOOTER + INTAKE ---------- */
        double targetVelocity = 1400;
        double intakePower = 0;

        if (gamepad2.cross) {               // X → intake reverse
            intakePower = -0.9;
        }
        else if (gamepad2.left_bumper) {    // LB → intake only
            intakePower = 1.0;
        }
        else if (gamepad2.right_bumper) {   // RB → shooter + intake + auto aim turret
            double distance = turretController.getDistanceToGoal(currentPose);
            targetVelocity = VELOCITY_SLOPE * distance + VELOCITY_INTERCEPT;
            intakePower = 1.0;
        }

        shooter1.setVelocity(targetVelocity);
        shooter2.setVelocity(targetVelocity);
        intake.setPower(intakePower);

        /* ---------- TELEMETRY ---------- */
        double targetAngle = turretController.calculateTurretAngle(currentPose);
        double currentAngle = turretController.getCurrentAngle();

        telemetry.addData("--- TURRET ---", "");
        telemetry.addData("Target Angle (0=right, 180=left)", "%.2f°", targetAngle);
        telemetry.addData("Current Angle", "%.2f°", currentAngle);
        telemetry.addData("Distance to Goal", "%.2f in", turretController.getDistanceToGoal(currentPose));

        telemetry.addData("--- BLOCKER ---", "");
        telemetry.addData("Status", gamepad2.triangle ? "OPEN" : "BLOCKING");
        telemetry.addData("Motor direction of front right", fr.getDirection());
        telemetry.update();
    }
}