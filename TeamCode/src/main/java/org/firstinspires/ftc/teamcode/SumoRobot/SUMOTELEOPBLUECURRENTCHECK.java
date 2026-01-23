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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


import java.util.concurrent.TimeUnit;

@Disabled
@TeleOp
public class SUMOTELEOPBLUECURRENTCHECK extends OpMode {


    private DcMotorEx fr, fl, br, bl;
    private DcMotorEx shooter1, shooter2;
    private DcMotorEx intake;
    private Servo kicker;

    private Follower follower;

    private final Pose startPose = new Pose(54, 8, Math.toRadians(90));
    private final Pose parkPose  = new Pose(104.67, 33, Math.toRadians(0));
    private final Pose shootFar = new Pose(63.7, 23, Math.toRadians(117));
    private final Pose shootClose = new Pose(60.1, 120.2, Math.toRadians(170));
    private final Pose emptyGate = new Pose(14.7, 70.5, Math.toRadians(270));


    private boolean kicking = false;
    private long kickStartTime = 0;
    private boolean kickerButtonLast = false;
    private static final long KICK_TIME = 130;

    private boolean holdingEmptyGate = false;
    private boolean holdingPark = false;
    private boolean holdingShootFar = false;
    private boolean holdingShootClose = false;

    public static final double LOW_VELOCITY  = 1500;
    public static final double HIGH_VELOCITY = 1850;

    public static final double KICKER_OUT = 0.52;
    public static final double KICKER_IN  = 0.35;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setMaxPower(1);

        // Changed to DcMotorEx for current monitoring
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        br = hardwareMap.get(DcMotorEx.class, "BR");
        bl = hardwareMap.get(DcMotorEx.class, "BL");

        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        intake = hardwareMap.get(DcMotorEx.class, "Intake");
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

        // Modified: Left bumper = single shooter (LOW_VELOCITY), Right bumper = both shooters (HIGH_VELOCITY)
        if (gamepad2.left_bumper) {
            // Low velocity - only shooter1
            shooter1.setVelocity(LOW_VELOCITY);
            shooter2.setVelocity(0);
            intake.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            // High velocity - both shooters
            shooter1.setVelocity(HIGH_VELOCITY);
            shooter2.setVelocity(HIGH_VELOCITY);
            intake.setPower(1.0);
        } else {
            // Off
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            intake.setPower(0.0);
        }

        // Get current draw from all motors (in Amps)
        double frCurrent = fr.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
        double flCurrent = fl.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
        double brCurrent = br.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
        double blCurrent = bl.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
        double shooter1Current = shooter1.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
        double shooter2Current = shooter2.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
        double intakeCurrent = intake.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);

        // Calculate totals
        double drivetrainCurrent = frCurrent + flCurrent + brCurrent + blCurrent;
        double shooterCurrent = shooter1Current + shooter2Current;
        double totalCurrent = drivetrainCurrent + shooterCurrent + intakeCurrent;

        // Telemetry
        telemetry.addData("Pose", follower.getPose());
        telemetry.addData("Holding Empty Gate", holdingEmptyGate);
        telemetry.addData("Holding Park", holdingPark);
        telemetry.addData("Holding Shoot Far", holdingShootFar);
        telemetry.addData("Holding Shoot Close", holdingShootClose);

        telemetry.addLine("\n--- DRIVETRAIN CURRENT (A) ---");
        telemetry.addData("FR", String.format("%.2f", frCurrent));
        telemetry.addData("FL", String.format("%.2f", flCurrent));
        telemetry.addData("BR", String.format("%.2f", brCurrent));
        telemetry.addData("BL", String.format("%.2f", blCurrent));
        telemetry.addData("Drivetrain Total", String.format("%.2f", drivetrainCurrent));

        telemetry.addLine("\n--- MECHANISM CURRENT (A) ---");
        telemetry.addData("Shooter1", String.format("%.2f", shooter1Current));
        telemetry.addData("Shooter2", String.format("%.2f", shooter2Current));
        telemetry.addData("Shooter Total", String.format("%.2f", shooterCurrent));
        telemetry.addData("Intake", String.format("%.2f", intakeCurrent));

        telemetry.addLine("\n--- TOTAL ---");
        telemetry.addData("All Motors", String.format("%.2f A", totalCurrent));

        // Warning if current is too high
        if (totalCurrent > 20.0) {
            telemetry.addLine("\n⚠️ WARNING: HIGH CURRENT DRAW!");
        }

        telemetry.update();
    }
}