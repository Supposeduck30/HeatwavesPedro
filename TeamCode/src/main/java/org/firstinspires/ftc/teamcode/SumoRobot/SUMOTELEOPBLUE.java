package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

@TeleOp
public class SUMOTELEOPBLUE extends OpMode {

    private DcMotor fr, fl, br, bl;
    private DcMotorEx shooter1, shooter2;
    private DcMotor intake;
    private Servo kicker;

    private Follower follower;

    private final Pose startPose = new Pose(54, 8, Math.toRadians(90));
    private final Pose parkPose  = new Pose(104.67, 33, Math.toRadians(0));
    private final Pose shootFar = new Pose(66, 18, Math.toRadians(115));
    private final Pose resetPose = new Pose(54.5, 8, Math.toRadians(90));
    private final Pose shootClose = new Pose(62, 108, Math.toRadians(148));
    private final Pose emptyGate = new Pose(14.7, 70.5, Math.toRadians(270));


    private boolean kicking = false;
    private long kickStartTime = 0;
    private boolean kickerButtonLast = false;
    private boolean resetButtonLast = false;
    private static final long KICK_TIME = 135;
    private boolean holdingEmptyGate = false;
    private boolean holdingPark = false;
    private boolean holdingShootFar = false;
    private boolean holdingShootClose = false;

    public static final double LOW_VELOCITY  = 1450;  // Close shot - shooter1 only
    public static final double HIGH_VELOCITY = 1800;  // Far shot - both shooters

    public static final double KICKER_OUT = 0.52;
    public static final double KICKER_IN  = 0.35;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(0.8);
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

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(90, 0, 0, 15);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        kicker.setPosition(KICKER_IN);
    }

    @Override
    public void start() {
        follower.setPose(startPose);
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();

        // Reset position button - Press TRIANGLE on gamepad1 to relocalize
        boolean resetButtonNow = gamepad1.triangle;
        if (resetButtonNow && !resetButtonLast) {
            // Break any ongoing path following
            follower.breakFollowing();

            // Reset all hold point flags
            holdingEmptyGate = false;
            holdingPark = false;
            holdingShootFar = false;
            holdingShootClose = false;

            // Relocalize to reset position
            follower.setPose(resetPose);

            // Restart teleop drive
            follower.startTeleopDrive();

            telemetry.addLine("⚠️ POSITION RESET!");
        }
        resetButtonLast = resetButtonNow;


        boolean leftBumperNow = gamepad1.left_bumper;
        boolean rightBumperNow = gamepad1.right_bumper;
        boolean squareNow = gamepad1.square;
        boolean circleNow = gamepad1.circle;

        // Open gate position (square button)
        if (squareNow && !holdingEmptyGate && !holdingPark && !holdingShootFar && !holdingShootClose) {
            follower.holdPoint(emptyGate);
            holdingEmptyGate = true;
        }

        if (!squareNow && holdingEmptyGate) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            holdingEmptyGate = false;
        }

        // Park position (circle button)
        if (circleNow && !holdingPark && !holdingEmptyGate && !holdingShootFar && !holdingShootClose) {
            follower.holdPoint(parkPose);
            holdingPark = true;
        }

        if (!circleNow && holdingPark) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            holdingPark = false;
        }

        // Handle shoot far position (left bumper)
        if (leftBumperNow && !holdingShootFar && !holdingEmptyGate && !holdingPark && !holdingShootClose) {
            follower.holdPoint(shootFar);
            holdingShootFar = true;
        }

        if (!leftBumperNow && holdingShootFar) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            holdingShootFar = false;
        }

        // Handle shoot close position (right bumper)
        if (rightBumperNow && !holdingShootClose && !holdingEmptyGate && !holdingPark && !holdingShootFar) {
            follower.holdPoint(shootClose);
            holdingShootClose = true;
        }

        if (!rightBumperNow && holdingShootClose) {
            follower.breakFollowing();
            follower.startTeleopDrive();
            holdingShootClose = false;
        }

        boolean holdingPoint = holdingEmptyGate || holdingPark || holdingShootFar || holdingShootClose;

        double driveForward = holdingPoint ? 0 : -gamepad1.left_stick_y;
        double driveStrafe  = holdingPoint ? 0 : -gamepad1.left_stick_x;
        double driveTurn    = holdingPoint ? 0 : -gamepad1.right_stick_x;

        follower.setTeleOpDrive(driveForward, driveStrafe, driveTurn, true);

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

        // Shooter and intake control - FIXED LOGIC
        if (gamepad2.right_bumper) {
            // Far shot - BOTH shooters run at HIGH_VELOCITY
            shooter1.setVelocity(HIGH_VELOCITY);
            shooter2.setVelocity(HIGH_VELOCITY);
            intake.setPower(1.0);
        } else if (gamepad2.left_bumper) {
            // Close shot - both shooters run at LOW_VELOCITY
            shooter1.setVelocity(LOW_VELOCITY);
            shooter2.setVelocity(LOW_VELOCITY);
            intake.setPower(1.0);
        } else if (gamepad2.x) {
            // Manual intake only (no shooting)
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            intake.setPower(1.0);
        } else {
            // Nothing pressed - turn everything off
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            intake.setPower(0.0);
        }


        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Holding Empty Gate", holdingEmptyGate);
        telemetry.addData("Holding Park", holdingPark);
        telemetry.addData("Holding Shoot Far", holdingShootFar);
        telemetry.addData("Holding Shoot Close", holdingShootClose);
        telemetry.addData("Shooter1 Velocity", shooter1.getVelocity());
        telemetry.addData("Shooter2 Velocity", shooter2.getVelocity());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.update();
    }
}