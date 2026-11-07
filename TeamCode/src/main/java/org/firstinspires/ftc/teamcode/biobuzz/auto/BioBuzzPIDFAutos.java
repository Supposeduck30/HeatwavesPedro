package org.firstinspires.ftc.teamcode.biobuzz.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * PIDF versions of the two BioBuzz autos (Red 1 and Blue 1). Same routes as {@link BioBuzzAutos}; the
 * logic is in {@link BioBuzzPIDFAutoBase}, the poses are in {@link BioBuzzAutoRoute}.
 */
public final class BioBuzzPIDFAutos {
    private BioBuzzPIDFAutos() {}

    @Autonomous(name = "PIDF Red 1 (audience wall, near hive)", group = "BioBuzz Auto")
    public static class Red1 extends BioBuzzPIDFAutoBase {
        public Red1() { super(BioBuzzAutoRoute.R1); }
    }

    @Autonomous(name = "PIDF Blue 1 (audience wall, near hive)", group = "BioBuzz Auto")
    public static class Blue1 extends BioBuzzPIDFAutoBase {
        // Shoot from (84.4044, 139.3705), then curve to the park pose (128.8908, 47.9048) through this control point.
        public Blue1() {
            super(BioBuzzAutoRoute.B1,
                    new Pose(84.4044, 139.3705, Math.toRadians(270)),
                    new Pose(124.1423, 118.0479, 0));
        }
    }
}
