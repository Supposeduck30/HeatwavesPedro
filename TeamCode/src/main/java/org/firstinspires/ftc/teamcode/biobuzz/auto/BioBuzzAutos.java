//package org.firstinspires.ftc.teamcode.biobuzz.auto;
//
//import com.pedropathing.math.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
///**
// * The two BioBuzz autos (Red 1 and Blue 1). Each one only picks a route; the logic is in
// * {@link BioBuzzAutoBase}, the poses are in {@link BioBuzzAutoRoute}.
// */
//public final class BioBuzzAutos {
//    private BioBuzzAutos() {}
//
//    @Autonomous(name = "Red 1 (audience wall, near hive)", group = "BioBuzz Auto")
//    public static class Red1 extends BioBuzzAutoBase {
//        public Red1() { super(BioBuzzAutoRoute.R1); }
//    }
//
//    @Autonomous(name = "Blue 1 (audience wall, near hive)", group = "BioBuzz Auto")
//    public static class Blue1 extends BioBuzzAutoBase {
//        // Same path as PIDF Blue 1: shoot from (84.4044, 139.3705), then curve to the park pose through this control point.
//        public Blue1() {
//            super(BioBuzzAutoRoute.B1,
//                    new Pose(84.4044, 139.3705, Math.toRadians(270)),
//                    new Pose(124.1423, 118.0479, 0));
//        }
//    }
//}
