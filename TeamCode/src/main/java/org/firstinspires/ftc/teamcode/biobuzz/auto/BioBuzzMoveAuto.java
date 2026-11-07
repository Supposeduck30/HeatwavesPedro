package org.firstinspires.ftc.teamcode.biobuzz.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.biobuzz.teleop.BioBuzzPose;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Move-only auto: follows one curve from the start pose to the end pose and holds there. No shooting.
 * Poses are in Pedro field inches, headings in degrees (from the Pedro Visualizer).
 */
@Autonomous(name = "Move Only (curve)", group = "BioBuzz Auto", preselectTeleOp = "Bang Bang TeleOp")
public class BioBuzzMoveAuto extends OpMode {

    private static final Pose START    = deg(57.8797, 134.3579, 270);
    private static final Pose CONTROL1 = deg(66.4981, 104.0545, 0);
    private static final Pose END      = deg(13.3934, 119.5424, 270);

    private Follower follower;
    private Path path1;
    private boolean done = false;

    private static Pose deg(double x, double y, double headingDeg) {
        return new Pose(x, y, Math.toRadians(headingDeg));
    }

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        path1 = Paths.curve(START, CONTROL1, END).linear(START, END);
        follower.setPose(START);

        telemetry.addLine("Place the robot on the start pose, then press play.");
        telemetry.update();
    }

    @Override
    public void start() {
        follower.setPose(START);
        follower.update();
        follower.follow(path1);
    }

    @Override
    public void loop() {
        follower.update();

        if (!done && follower.atParametricEnd()) {
            follower.hold(END);
            BioBuzzPose.savePose(follower.pose());
            done = true;
        }

        Pose pose = follower.pose();
        telemetry.addData("State", done ? "DONE" : "DRIVING");
        telemetry.addData("X / Y (in)", "%.2f / %.2f", pose.x(), pose.y());
        telemetry.addData("Heading (deg)", "%.2f", Math.toDegrees(pose.heading()));
        telemetry.update();
    }

    @Override
    public void stop() {
        // Hand the final pose to the teleop even if the auto ended early.
        BioBuzzPose.savePose(follower.pose());
    }
}
