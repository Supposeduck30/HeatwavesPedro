package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.pedropathing.math.Pose;

/**
 * Static storage that survives between OpModes so teleop can start where auto ended
 * (Pedro 3 port of Decode's PedroPose).
 *
 * In an autonomous, at the end (e.g. in stop()):
 * <pre>
 *     BioBuzzPose.save(follower.pose(), turret.getRawTicks());
 * </pre>
 * BioBuzzTeleOp reads it in init() and falls back to the alliance start pose if nothing was saved.
 * Static state is lost if the robot controller app restarts, which is fine: you get the fallback.
 */
public final class BioBuzzPose {
    private BioBuzzPose() {}

    private static Pose lastKnownPose = null;
    private static Integer lastKnownTurretTicks = null;

    public static void save(Pose pose, int turretTicks) {
        lastKnownPose = pose;
        lastKnownTurretTicks = turretTicks;
    }

    public static void savePose(Pose pose) {
        lastKnownPose = pose;
    }

    public static void saveTurretTicks(int ticks) {
        lastKnownTurretTicks = ticks;
    }

    public static boolean hasPoseFromAuto() {
        return lastKnownPose != null;
    }

    public static Pose getPose() {
        return lastKnownPose;
    }

    /** Raw turret ticks at the end of auto, or null if auto never saved them. */
    public static Integer getTurretTicks() {
        return lastKnownTurretTicks;
    }

    public static void clear() {
        lastKnownPose = null;
        lastKnownTurretTicks = null;
    }
}
