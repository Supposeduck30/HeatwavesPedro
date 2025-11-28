package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoPath1", group = "Autonomous")
@Configurable
public class AutoPath1 extends OpMode {

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState = 0;    // Start at state 0
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
    public void start() {
        pathState = 0; // Reset state machine on start
    }

    @Override
    public void loop() {
        follower.update();

        pathState = autonomousPathUpdate();

        // Telemetry for Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    // -------------------------
    //   PATH CLASS
    // -------------------------
    public static class Paths {

        public PathChain line1;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 8.000), new Pose(60.702, 80.862))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120.3))
                    .build();
        }
    }

    // -----------------------------------------
    //          STATE MACHINE LOGIC
    // -----------------------------------------
    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:   // Start line1
                follower.followPath(paths.line1);
                return 1;

            case 1:   // Wait until line1 is finished
                if (!follower.isBusy()) {
                    return 2;
                }
                return 1;

            case 2:   // Auto complete
                // Optional: stop robot or add scoring actions here
                return 2;
        }
        return pathState;
    }
}
