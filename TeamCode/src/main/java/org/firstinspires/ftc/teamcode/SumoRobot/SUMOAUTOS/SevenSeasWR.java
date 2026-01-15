package org.firstinspires.ftc.teamcode.SumoRobot.SUMOAUTOS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "Seven Seas WR")
@Configurable
public class SevenSeasWR extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;

    private int pathState;
    private Paths paths;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower);

        pathState = 0;
        follower.followPath(paths.Shoot1);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    pathState++;
                    follower.followPath(paths.IntakeRow1);
                }
                break;

            case 1:
                if (!follower.isBusy()) {
                    pathState++;
                    follower.followPath(paths.Shoot2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    pathState++;
                    follower.followPath(paths.IntakeOverflow1);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathState++;
                    follower.followPath(paths.Shoot3);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    pathState++;
                    follower.followPath(paths.IntakeOverflow2);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    pathState++;
                    follower.followPath(paths.Shoot4);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    pathState++;
                    follower.followPath(paths.IntakeOverflow3);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    pathState++;
                    follower.followPath(paths.Shoot5);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    pathState++;
                    // AUTO COMPLETE
                }
                break;
        }
    }

    public static class Paths {

        public PathChain Shoot1;
        public PathChain IntakeRow1;
        public PathChain Shoot2;
        public PathChain IntakeOverflow1;
        public PathChain Shoot3;
        public PathChain IntakeOverflow2;
        public PathChain Shoot4;
        public PathChain IntakeOverflow3;
        public PathChain Shoot5;

        public Paths(Follower follower) {

            Shoot1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(56.000, 8.000),
                            new Pose(60.200, 19.900)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(119))
                    .build();

            IntakeRow1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.200, 19.900),
                            new Pose(66.043, 38.244),
                            new Pose(38.075, 31.686),
                            new Pose(11.706, 36.442)))
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                    .build();

            Shoot2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(11.706, 36.442),
                            new Pose(60.300, 19.900)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(119))
                    .build();

            IntakeOverflow1 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.300, 19.900),
                            new Pose(4.700, 81.800),
                            new Pose(6.400, 6.700)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot3 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(6.400, 6.700),
                            new Pose(60.200, 20.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(119))
                    .build();

            IntakeOverflow2 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.200, 20.000),
                            new Pose(31.029, 3.150),
                            new Pose(1.876, 0.242),
                            new Pose(8.693, 26.902)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot4 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(8.693, 26.902),
                            new Pose(35.447, 26.451),
                            new Pose(60.200, 20.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(119))
                    .build();

            IntakeOverflow3 = follower.pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(60.200, 20.000),
                            new Pose(40.940, 4.258),
                            new Pose(26.597, 6.583),
                            new Pose(2.302, 2.620),
                            new Pose(12.822, 37.804)))
                    .setTangentHeadingInterpolation()
                    .build();

            Shoot5 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(12.822, 37.804),
                            new Pose(60.200, 20.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(119))
                    .build();
        }
    }
}
