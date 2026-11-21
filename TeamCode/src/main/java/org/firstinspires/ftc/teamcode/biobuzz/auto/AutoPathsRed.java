package org.firstinspires.ftc.teamcode.biobuzz.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Pedro Pathing visualizer export used by {@link BioBuzzAutoSolo2Red}. The class is called AutoPathsRed (not Paths)
 * so it doesn't clash with com.pedropathing.api.Paths. Pedro 3 style: Paths.line / Paths.curve ... .linear(a, b).
 */
public class AutoPathsRed {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(54.6158, 7.3815, 180);
    private final Pose intakepresetEnd = poseFactory.of(8.3542, 7.0708, 180);
    private final Pose shootpresetStart = poseFactory.of(8.3542, 7.0708, 180);
    private final Pose shootpresetEnd = poseFactory.of(62.2484, 126.682, 90);
    private final Pose shootpresetControl1 = poseFactory.of(51.2527, 31.5047, 0);
    private final Pose shootpresetControl2 = poseFactory.of(6.6855, 110.394, 0);
    private final Pose intakeflower1Start = poseFactory.of(62.2484, 126.682, 90);
    private final Pose intakeflower1End = poseFactory.of(46.6212, 127.5466, 90);
    private final Pose intakeflower1Control1 = poseFactory.of(55.7299, 93.6497, 0);
    private final Pose shootflower1Start = poseFactory.of(46.6212, 127.5466, 90);
    private final Pose shootflower1End = poseFactory.of(62.152, 126.3463, 180);
    private final Pose shootflower1Control1 = poseFactory.of(51.7862, 111.1408, 0);
    private final Pose gotoflowerStart = poseFactory.of(62.152, 126.3463, 180);
    private final Pose gotoflowerEnd = poseFactory.of(22.2794, 47.4389, 270);
    private final Pose gotoflowerControl1 = poseFactory.of(23.2985, 98.3109, 0);
    private final Pose intakeflower2Start = poseFactory.of(22.2794, 47.4389, 180);
    private final Pose intakeflower2End = poseFactory.of(13.1353, 47.005, 180);
    private final Pose shootflower2Start = poseFactory.of(13.1353, 47.005, 180);
    private final Pose shootflower2End = poseFactory.of(62.5436, 126.1287, 270);
    private final Pose shootflower2Control1 = poseFactory.of(30.3517, 107.9602, 0);
    private final Pose point8Start = poseFactory.of(62.5436, 126.1287, 270);
    private final Pose point8End = poseFactory.of(14.7048, 122.0856, 270);

    public Path intakepreset;
    public Path shootpreset;
    public Path intakeflower1;
    public Path shootflower1;
    public Path gotoflower;
    public Path intakeflower2;
    public Path shootflower2;
    public Path path8;

    public AutoPathsRed(Follower follower) {
        intakepreset  = Paths.line(start, intakepresetEnd).linear(start, intakepresetEnd);
        shootpreset   = Paths.curve(shootpresetStart, shootpresetControl1, shootpresetControl2, shootpresetEnd).linear(shootpresetStart, shootpresetEnd);
        intakeflower1 = Paths.curve(intakeflower1Start, intakeflower1Control1, intakeflower1End).linear(intakeflower1Start, intakeflower1End);
        shootflower1  = Paths.curve(shootflower1Start, shootflower1Control1, shootflower1End).linear(shootflower1Start, shootflower1End);
        gotoflower    = Paths.curve(gotoflowerStart, gotoflowerControl1, gotoflowerEnd).linear(gotoflowerStart, gotoflowerEnd);
        intakeflower2 = Paths.line(intakeflower2Start, intakeflower2End).linear(intakeflower2Start, intakeflower2End);
        shootflower2  = Paths.curve(shootflower2Start, shootflower2Control1, shootflower2End).linear(shootflower2Start, shootflower2End);
        path8         = Paths.line(point8Start, point8End).linear(point8Start, point8End);
    }

    /** True once every path has been built. */
    public boolean isComplete() {
        return intakepreset != null && shootpreset != null && intakeflower1 != null && shootflower1 != null
                && gotoflower != null && intakeflower2 != null && shootflower2 != null && path8 != null;
    }
}
