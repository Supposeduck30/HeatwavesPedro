package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.pedropathing.math.Pose;

/**
 * Everything that differs between the red and blue teleops (Decode had a separate teleop and
 * turret class for each). All values are the Decode numbers and need re-tuning for BioBuzz.
 *
 * The BioBuzz field is symmetric by a 180 deg rotation about the center (loading zones A5 / F2,
 * gardens on opposite walls), not a mirror. RED is the tuned side; every BLUE pose is the RED one
 * with x -> 144-x, y -> 144-y, heading + 180, same as the blue autos in BioBuzzAutoRoute.
 * Change a RED pose and update its BLUE line to match.
 */
public enum BioBuzzAlliance {

    //          primary goal    secondary goal   start pose (x, y, heading deg)   park pose (x, y, heading deg)   vel regression A, B, C         start trim   reset pose (gamepad1 triangle)   relocalize pose (gamepad1 dpad up)   shoot far pose (left bumper)   shoot close pose (right bumper)
    BLUE(  6.2, 187.2,    86.8, 184.2,    144 - 54.4, 144 - 7, 270,   144 - 38.8, 144 - 31.5, 180,   0.0220573, -0.464531, 1402.823,   0.0,
            new Pose(144 - 7, 144 - 9, Math.toRadians(270)),
            new Pose(133.9, 134.58, Math.toRadians(270)),
            new Pose(144 - 60.35, 144 - 7, Math.toRadians(270)),  new Pose(144 - 60.35, 144 - 137, Math.toRadians(90))),
    RED( 180.1, 187.2,   100.0, 185.0,   54.4, 7, 90,            38.8, 31.5, 0,     0.0220573, -0.464531, 1402.823,   0.0,
            new Pose(7, 9, Math.toRadians(90)),
            new Pose(7.75, 8.01, Math.toRadians(90)),
            new Pose(60.35, 7, Math.toRadians(90)),  new Pose(60.35, 137, Math.toRadians(270)));  // same x, each 7 in from its wall (was 59.09, 7 and 61.60, 119.46)

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
    /** Starting turret aim trim in degrees. */
    public final double startTrimDeg;

    BioBuzzAlliance(double primaryGoalX, double primaryGoalY,
                    double secondaryGoalX, double secondaryGoalY,
                    double startX, double startY, double startHeadingDeg,
                    double parkX, double parkY, double parkHeadingDeg,
                    double velA, double velB, double velC,
                    double startTrimDeg,
                    Pose resetPose, Pose relocalizePose, Pose shootFarPose, Pose shootClosePose) {
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

    public double shooterVelocity(double distanceInches) {
        return velA * distanceInches * distanceInches + velB * distanceInches + velC;
    }
}
