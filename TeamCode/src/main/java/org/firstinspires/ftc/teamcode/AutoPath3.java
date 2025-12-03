package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@Autonomous(name = "AutoPath3", group = "Autonomous")
@Configurable
public class AutoPath3 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;          // Start at state 0
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();

        // Telemetry updates
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // ----------------------------
    // PATH DEFINITIONS
    // ----------------------------
    public static class Paths {
        public PathChain MoveBack;
        public PathChain Leave;

        public Paths(Follower follower) {

            MoveBack = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(24.435, 125.970),
                            new Pose(71.170, 71.644)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(145),
                            Math.toRadians(138))
                    .build();

            Leave = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(71.170, 71.644),
                            new Pose(60.257, 112.923)))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(138),
                            Math.toRadians(90))
                    .build();
        }
    }

    // ----------------------------
    // STATE MACHINE
    // ----------------------------
    public int autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                // Start moving back
                follower.followPath(paths.MoveBack);
                pathState = 1;
                break;

            case 1:
                // Wait until path finishes
                if (!follower.isBusy()) {
                    follower.followPath(paths.Leave);
                    pathState = 2;
                }
                break;

            case 2:
                // Done
                if (!follower.isBusy()) {
                    pathState = 3; // Finished state
                }
                break;

            case 3:
                // Autonomous finished
                break;
        }

        return pathState;
    }
}
