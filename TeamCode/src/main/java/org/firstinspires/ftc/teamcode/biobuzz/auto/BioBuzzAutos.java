package org.firstinspires.ftc.teamcode.biobuzz.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.biobuzz.shooter.PIDFShooter;

/**
 * The PIDF BioBuzz route autos (Red 1 and Blue 1). Each one only picks a route and the shooter control; the
 * logic is in {@link BioBuzzAutoBase}, the poses are in {@link BioBuzzAutoRoute}.
 */
public final class BioBuzzAutos {
    private BioBuzzAutos() {}

    /** Red 1: curve from the shoot pose to the park pose through this control point. */
    private static final Pose RED1_PARK_CONTROL = new Pose(20, 20, 0);

    @Autonomous(name = "Red 1 (audience wall, near hive)", group = "BioBuzz Auto")
    public static class PIDFRed1 extends BioBuzzAutoBase {
        public PIDFRed1() { super(BioBuzzAutoRoute.R1, new PIDFShooter(), null, RED1_PARK_CONTROL); }
    }

    @Autonomous(name = "Blue 1 (audience wall, near hive)", group = "BioBuzz Auto")
    public static class PIDFBlue1 extends BioBuzzAutoBase {
        // Shoot from (84.4044, 139.3705), then curve to the park pose (128.8908, 47.9048) through this control point.
        public PIDFBlue1() {
            super(BioBuzzAutoRoute.B1, new PIDFShooter(),
                    new Pose(84.4044, 139.3705, Math.toRadians(270)),
                    new Pose(124.1423, 118.0479, 0));
        }
    }
}
