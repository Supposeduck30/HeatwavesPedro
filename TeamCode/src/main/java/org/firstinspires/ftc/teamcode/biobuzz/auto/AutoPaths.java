package org.firstinspires.ftc.teamcode.biobuzz.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * Pedro Pathing visualizer export used by {@link BioBuzzAutoSolo2Blue}. The class is called AutoPaths (not Paths)
 * so it doesn't clash with com.pedropathing.api.Paths. Pedro 3 style: Paths.line / Paths.curve ... .linear(a, b).
 */
public class AutoPaths {

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(86.7161, 133.9505, 0);
    private final Pose intakepresetEnd = poseFactory.of(133.5377, 133.5563, 0);
    private final Pose shootpresetStart = poseFactory.of(133.5377, 133.5563, 0);
    private final Pose shootpresetEnd = poseFactory.of(80.9734, 12.6123, 270);
    private final Pose shootpresetControl1 = poseFactory.of(113.1237, 86.4051, 0);
    private final Pose shootpresetControl2 = poseFactory.of(122.0788, 17.9562, 0);
    private final Pose intakeflower1Start = poseFactory.of(80.9734, 12.6123, 270);
    private final Pose intakeflower1End = poseFactory.of(94.4276, 11.605, 270);
    private final Pose intakeflower1Control1 = poseFactory.of(92.9112, 40.4155, 0);
    private final Pose shootflower1Start = poseFactory.of(94.4276, 11.605, 270);
    private final Pose shootflower1End = poseFactory.of(81.3933, 11.9737, 0);
    private final Pose shootflower1Control1 = poseFactory.of(92.2138, 33.7024, 0);
    private final Pose gotoflowerStart = poseFactory.of(81.3933, 11.9737, 0);
    private final Pose gotoflowerEnd = poseFactory.of(121.2991, 95.7179, 90);
    private final Pose gotoflowerControl1 = poseFactory.of(120.7015, 45.6891, 0);
    private final Pose intakeflower2Start = poseFactory.of(121.2991, 95.7179, 0);
    private final Pose intakeflower2End = poseFactory.of(130.8647, 95.5194, 0);
    private final Pose shootflower2Start = poseFactory.of(130.8647, 95.5194, 0);
    private final Pose shootflower2End = poseFactory.of(82.5579, 11.1101, 90);
    private final Pose shootflower2Control1 = poseFactory.of(108.6357, 37.5018, 0);
    private final Pose point8Start = poseFactory.of(82.5579, 11.1101, 90);
    private final Pose point8End = poseFactory.of(129.2952, 21.9144, 90);

    public Path intakepreset;
    public Path shootpreset;
    public Path intakeflower1;
    public Path shootflower1;
    public Path gotoflower;
    public Path intakeflower2;
    public Path shootflower2;
    public Path path8;

    public AutoPaths(Follower follower) {
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
