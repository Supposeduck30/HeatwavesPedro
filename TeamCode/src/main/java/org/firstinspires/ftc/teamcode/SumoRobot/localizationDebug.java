package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "DEBUG: Localization Test", group = "Debug")
public class localizationDebug extends OpMode {

    private Follower follower;
    private DcMotor fr, fl, br, bl;

    private Pose lastPose = null;
    private long lastTime = 0;
    private double peakSpeed = 0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));

        fr = hardwareMap.get(DcMotor.class, "FR");
        fl = hardwareMap.get(DcMotor.class, "FL");
        br = hardwareMap.get(DcMotor.class, "BR");
        bl = hardwareMap.get(DcMotor.class, "BL");

        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);

        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Localization Test Ready.");
        telemetry.addLine("Drive around, then return to start and check drift.");
        telemetry.addLine("A = reset pose to (0,0,0)");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
        lastTime = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        follower.update();

        if (gamepad1.a) {
            follower.setPose(new Pose(0, 0, 0));
            peakSpeed = 0;
        }

        double forward = -gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double turn    = -gamepad1.right_stick_x;
        follower.setTeleOpDrive(forward, strafe, turn, true);

        Pose pose = follower.getPose();
        long now = System.currentTimeMillis();
        double dt = (now - lastTime) / 1000.0;
        if (dt <= 0) dt = 0.001;

        // Manual velocity from pose delta
        double velX = 0, velY = 0;
        if (lastPose != null && dt > 0.005) {
            velX = (pose.getX() - lastPose.getX()) / dt;
            velY = (pose.getY() - lastPose.getY()) / dt;
        }

        double speed = Math.sqrt(velX * velX + velY * velY);
        if (speed > peakSpeed) peakSpeed = speed;

        double headingDeg    = Math.toDegrees(pose.getHeading());
        double distFromStart = Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY());

        // ---- TELEMETRY ----
        telemetry.addLine("=== POSITION ===");
        telemetry.addData("X (inches)",         "%.3f", pose.getX());
        telemetry.addData("Y (inches)",         "%.3f", pose.getY());
        telemetry.addData("Heading (degrees)",  "%.3f", headingDeg);
        telemetry.addData("Heading (radians)",  "%.4f", pose.getHeading());

        telemetry.addLine("\n=== DRIFT CHECK ===");
        telemetry.addLine("Drive back to start, then read below.");
        telemetry.addLine("Ideal: X=0, Y=0, Heading=0");
        telemetry.addData("Distance from origin", "%.3f in", distFromStart);

        telemetry.addLine("\n=== VELOCITY ===");
        telemetry.addData("Vel X",      "%.2f in/s", velX);
        telemetry.addData("Vel Y",      "%.2f in/s", velY);
        telemetry.addData("Speed",      "%.2f in/s", speed);
        telemetry.addData("Peak Speed", "%.2f in/s", peakSpeed);

        telemetry.addLine("\n=== HEADING DETAIL ===");
        telemetry.addData("Raw heading",        "%.4f rad", pose.getHeading());
        telemetry.addData("Normalized heading", "%.2f°", headingDeg);
        telemetry.addLine("Spin 360° and return — heading should read ~0°");

        telemetry.addLine("\n=== CONTROLS ===");
        telemetry.addData("A", "Reset pose + peak speed to (0,0,0)");

        telemetry.update();

        lastPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
        lastTime = now;
    }
}