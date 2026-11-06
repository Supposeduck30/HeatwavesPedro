package org.firstinspires.ftc.teamcode.biobuzz.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * The six BioBuzz autos. Each one only picks a route; the logic is in {@link BioBuzzAutoBase},
 * the poses are in {@link BioBuzzAutoRoute}.
 */
public final class BioBuzzAutos {
    private BioBuzzAutos() {}

    @Autonomous(name = "Red 1 (audience wall, near hive)", group = "BioBuzz Auto")
    public static class Red1 extends BioBuzzAutoBase {
        public Red1() { super(BioBuzzAutoRoute.R1); }
    }

    @Autonomous(name = "Red 2 (audience-wall corner)", group = "BioBuzz Auto")
    public static class Red2 extends BioBuzzAutoBase {
        public Red2() { super(BioBuzzAutoRoute.R2); }
    }

    @Autonomous(name = "Red 3 (far wall, left of hive)", group = "BioBuzz Auto")
    public static class Red3 extends BioBuzzAutoBase {
        public Red3() { super(BioBuzzAutoRoute.R3); }
    }

    @Autonomous(name = "Blue 1 (audience wall, near hive)", group = "BioBuzz Auto")
    public static class Blue1 extends BioBuzzAutoBase {
        public Blue1() { super(BioBuzzAutoRoute.B1); }
    }

    @Autonomous(name = "Blue 2 (audience-wall corner)", group = "BioBuzz Auto")
    public static class Blue2 extends BioBuzzAutoBase {
        public Blue2() { super(BioBuzzAutoRoute.B2); }
    }

    @Autonomous(name = "Blue 3 (far wall, right of hive)", group = "BioBuzz Auto")
    public static class Blue3 extends BioBuzzAutoBase {
        public Blue3() { super(BioBuzzAutoRoute.B3); }
    }
}
