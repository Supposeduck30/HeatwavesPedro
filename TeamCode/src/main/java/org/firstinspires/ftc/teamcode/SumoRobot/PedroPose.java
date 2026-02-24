package org.firstinspires.ftc.teamcode.SumoRobot;

import com.pedropathing.geometry.Pose;

/**
 * Static storage for robot pose that persists between OpModes.
 * Allows TeleOp to know where Autonomous actually ended.
 */
public class PedroPose {

    // --- ADD THESE VARIABLES ---
    private static Integer lastKnownTurretTicks = null;

    // --- ADD THESE METHODS ---
    public static void saveTurretTicks(int ticks) {
        lastKnownTurretTicks = ticks;
    }

    public static Integer getTurretTicks() {
        return lastKnownTurretTicks;
    }

    // --- UPDATE YOUR EXISTING CLEAR METHOD ---



    // Static pose that persists between OpModes
    private static Pose lastKnownPose = null;
    private static boolean poseSetByAuto = false;

    /**
     * Save the current pose (call this in autonomous at the end)
     * @param pose The final pose from autonomous
     */
    public static void saveCurrentPose(Pose pose) {
        lastKnownPose = pose;
        poseSetByAuto = true;
    }

    /**
     * Get the last known pose from autonomous
     * @return The saved pose, or null if no pose has been saved
     */
    public static Pose getLastKnownPose() {
        return lastKnownPose;
    }

    /**
     * Check if a pose was set by autonomous
     * @return true if a pose from auto is available
     */
    public static boolean hasPoseFromAuto() {
        return poseSetByAuto && lastKnownPose != null;
    }

    /**
     * Clear the saved pose (optional - for debugging)
     */
    public static void clearSavedPose() {
        lastKnownPose = null;
        poseSetByAuto = false;
        lastKnownTurretTicks=null;
    }

    /**
     * Get a default starting pose for TeleOp if auto didn't run
     * Adjust these values to match your typical autonomous end positions
     * @return Default pose for TeleOp start
     */
    public static Pose getDefaultTeleOpStartPose() {
        // Default
        return new Pose(50, 60, Math.toRadians(270));
    }

    /**
     * Get the appropriate starting pose for TeleOp
     * Uses auto end pose if available, otherwise uses default
     * @return The best starting pose for TeleOp
     */
    public static Pose getTeleOpStartPose() {
        if (hasPoseFromAuto()) {
            return lastKnownPose;
        } else {
            return getDefaultTeleOpStartPose();
        }
    }
}

/*
 * USAGE GUIDE:
 *
 * === IN AUTONOMOUS ===
 * At the end of your autonomous, add to stop() method:
 *
 * @Override
 * public void stop() {
 *     Pose finalPose = follower.getPose();
 *     PedroPose.saveCurrentPose(finalPose);
 *     super.stop();
 * }
 *
 * === IN TELEOP ===
 * In your init(), use the stored pose:
 *
 * @Override
 * public void init() {
 *     follower = Constants.createFollower(hardwareMap);
 *
 *     // Get starting pose - either from auto or default
 *     Pose startPose = PedroPose.getTeleOpStartPose();
 *     follower.setStartingPose(startPose);

 *     telemetry.addData("Start Pose", "(%.1f, %.1f, %.1f°)",
 *         startPose.getX(), startPose.getY(), Math.toDegrees(startPose.getHeading()));
 *     telemetry.update();
 */