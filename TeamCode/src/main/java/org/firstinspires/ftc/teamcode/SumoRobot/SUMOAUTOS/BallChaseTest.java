package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Ball Chase Test", group = "Auto")
public class BallChaseTest extends OpMode {

    private Follower follower;
    private Limelight3A limelight;

    private final Pose startPose = new Pose(0, 0, 0);
    private boolean pathStarted = false;

    // ---------------- INIT ----------------
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setPose(startPose);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);
        limelight.start();
    }

    // ---------------- START ----------------
    @Override
    public void start() {
        // give Limelight a moment to settle
        resetRuntime();
    }

    // ---------------- LOOP ----------------
    @Override
    public void loop() {

        // Only decide once
        if (!pathStarted && getRuntime() > 0.3) {
            if (ballDetected()) {
                Pose endPose = new Pose(startPose.getX() + 12, startPose.getY(), startPose.getHeading());

                PathChain forwardPath = follower.pathBuilder()
                        .addPath(new BezierLine(startPose, endPose))
                        .setConstantHeadingInterpolation(startPose.getHeading())
                        .build();

                follower.followPath(forwardPath, true);
            }
            pathStarted = true;
        }

        follower.update();
    }

    // ---------------- VISION ----------------
    private boolean ballDetected() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            double ta = result.getTa();
            return ta > 0.4; // tune threshold
        }
        return false;
    }
}