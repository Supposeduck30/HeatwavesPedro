package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.pedropathing.math.Pose;

/**
 * Everything that differs between the red and blue teleops (Decode had a separate teleop and
 * turret class for each). All values are the Decode numbers and need re-tuning for BioBuzz.
 */
public enum BioBuzzAlliance {

    //          primary goal    secondary goal   start pose (x, y, heading deg)   park pose (x, y, heading deg)   vel regression A, B, C         start trim   shoot far pose (left bumper)   shoot close pose (right bumper)
    BLUE(  6.2, 187.2,    86.8, 184.2,    78.7, 8, 90,                105.3, 31.8, 0,    0.016966, 1.45242, 1235.32337,    3.5,
            new Pose(83.23753971451379, 21.328998448498826, Math.toRadians(90)),  new Pose(83.81706779401863, 118.8818994723896, Math.toRadians(270))),
    RED( 180.1, 187.2,   100.0, 185.0,   112.19, 9.25, 90,             38.8, 31.5, 0,     0.0220573, -0.464531, 1402.823,   0.0,
            new Pose(59.09079677334417, 24.612955282165725, Math.toRadians(90)),  new Pose(61.60205809817785, 119.46142018288882, Math.toRadians(270)));

    public final double primaryGoalX, primaryGoalY;
    public final double secondaryGoalX, secondaryGoalY;
    public final Pose startPose;
    public final Pose parkPose;
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
                    Pose shootFarPose, Pose shootClosePose) {
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
