package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.pedropathing.math.Pose;

/**
 * Everything that differs between the red and blue teleops (Decode had a separate teleop and
 * turret class for each). The poses are BioBuzz field positions.
 *
 * The BioBuzz field is symmetric by a 180 deg rotation about the center (loading zones A5 / F2,
 * gardens on opposite walls), not a mirror. RED is the tuned side; every BLUE pose is the RED one
 * with x -> 144-x, y -> 144-y, heading + 180, same as the blue autos in BioBuzzAutoRoute.
 * Change a RED pose and update its BLUE line to match.
 */
public enum BioBuzzAlliance {

    // GOALS (primary, secondary): the y split is 70, and the teleop switches
    // between them by the robot's y (autoGoalSplitY = 70). The field is rotated 180 deg, not mirrored, so the rule differs:
    //   RED : y > 70 -> secondary (57.47, 74.7),  y <= 70 -> primary (59.47, 65.3)
    //   BLUE: y <= 70 -> secondary (83.78, 62.506), y > 70  -> primary (83.78, 77.494)   (see useSecondaryGoalAt)
    //          primary goal    secondary goal   start pose (x, y, heading deg)   park pose (x, y, heading deg)   vel regression A, B, C         start trim   reset pose (gamepad1 triangle)   relocalize pose (gamepad1 dpad up)   shoot far pose (left bumper)   shoot close pose (right bumper)
    // Start pose = the Solo2 auto start SPOT (BioBuzzAutoSolo2Blue/Red START_X/Y), used by teleop when no auto ran first.
    // Heading = the robot facing UP the field toward its goal (Red 90, Blue 270), NOT the auto's 180 / 0 (that is the
    // robot placed sideways). The heading must match how the robot is set down, or the turret aims off by the difference.
    // (x/y were swapped before 2026-11-24: Red teleop thought it was at the blue start spot.)
    BLUE(  83.78, 77.494,    83.78, 62.506,    86.7161, 133.9505, 270,     38.8, 31.5, 0,     -0.251195, 36.6639, 177.5342,   0.0,
            new Pose(7, 9, Math.toRadians(90)),
            new Pose(57.6, 133.8, Math.toRadians(270)),
            new Pose(60.35, 7, Math.toRadians(90)),  new Pose(60.35, 137, Math.toRadians(270)),
            70.0, false),
    RED( 59.47, 65.3,   57.47, 74.7,  54.6158, 7.3815, 90,   144 - 38.8, 144 - 31.5, 180,   -0.251195, 36.6639, 177.5342,   0.0,
            new Pose(144 - 7, 144 - 9, Math.toRadians(270)),
            new Pose(144 - 57.6, 144 - 133.8, Math.toRadians(90)),
            new Pose(144 - 60.35, 144 - 7, Math.toRadians(270)),  new Pose(144 - 60.35, 144 - 137, Math.toRadians(90)),
            70.0, true);  // same x, each 7 in from its wall (was 59.09, 7 and 61.60, 119.46)
    public final double primaryGoalX, primaryGoalY;
    public final double secondaryGoalX, secondaryGoalY;
    public final Pose startPose;
    public final Pose parkPose;
    /** gamepad1 triangle snaps the robot to this pose (use against a wall to fix drift). */
    public final Pose resetPose;
    /** gamepad1 dpad up relocalizes the Pedro follower to this pose (drive to this spot, press, and odometry restarts from here). */
    public final Pose relocalizePose;
    /** gamepad1 left bumper (hold) drives to and holds this pose. */
    public final Pose shootFarPose;
    /** gamepad1 right bumper (hold) drives to and holds this pose. */
    public final Pose shootClosePose;
    /** Shooter velocity (ticks/s) = A*d^2 + B*d + C, d = distance to goal in inches. */
    public final double velA, velB, velC;
    /**
     * Teleop picks the goal by the robot's y: above this = secondary goal, at or below = primary goal (gamepad2 circle is
     * ignored). NaN = no automatic choice, circle toggles the goal by hand.
     */
    public final double autoGoalSplitY;
    /** true = the secondary goal is used ABOVE the split y (Red); false = at or BELOW it (Blue). */
    public final boolean secondaryAboveSplit;
    /** Starting turret aim trim in degrees. */
    public final double startTrimDeg;

    BioBuzzAlliance(double primaryGoalX, double primaryGoalY,
                    double secondaryGoalX, double secondaryGoalY,
                    double startX, double startY, double startHeadingDeg,
                    double parkX, double parkY, double parkHeadingDeg,
                    double velA, double velB, double velC,
                    double startTrimDeg,
                    Pose resetPose, Pose relocalizePose, Pose shootFarPose, Pose shootClosePose,
                    double autoGoalSplitY, boolean secondaryAboveSplit) {
        this.autoGoalSplitY = autoGoalSplitY;
        this.secondaryAboveSplit = secondaryAboveSplit;
        this.resetPose = resetPose;
        this.relocalizePose = relocalizePose;
        this.shootFarPose = shootFarPose;
        this.shootClosePose = shootClosePose;
        this.primaryGoalX = primaryGoalX;
        this.primaryGoalY = primaryGoalY;
        this.secondaryGoalX = secondaryGoalX;
        this.secondaryGoalY = secondaryGoalY;
        this.startPose = new Pose(startX, startY, Math.toRadians(startHeadingDeg));
        this.parkPose = new Pose(parkX, parkY, Math.toRadians(parkHeadingDeg));
        this.velA = velA;
        this.velB = velB;
        this.velC = velC;
        this.startTrimDeg = startTrimDeg;
    }

    /** Which goal to aim at for a robot at this y: true = secondary. Only meaningful when autoGoalSplitY is not NaN. */
    public boolean useSecondaryGoalAt(double robotY) {
        return secondaryAboveSplit ? robotY > autoGoalSplitY : robotY <= autoGoalSplitY;
    }

    public double shooterVelocity(double distanceInches) {
        return velA * distanceInches * distanceInches + velB * distanceInches + velC;
    }
}
