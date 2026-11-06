package org.firstinspires.ftc.teamcode.biobuzz.auto;

import com.pedropathing.math.Pose;

/**
 * The six BioBuzz autonomous routes: start -> shoot -> park, every segment a straight line.
 * Poses are in Pedro field inches (origin bottom-left, +x right, +y up), headings in degrees.
 * Blue is the red layout rotated 180 deg: x -> 144-x, y -> 144-y, heading + 180.
 *
 * These were designed from the manual, not read from the Pedro Visualizer, so measure on the real field.
 */
public enum BioBuzzAutoRoute {

    //   start (x, y, heading deg)    shoot (x, y, heading deg)     park (x, y, heading deg)
    // R1/B1: no turn and no trip across the field. The shoot pose is the start pose strafed ~6 in to the
    // robot's right (same heading) to line up with the goal; then it parks at (137, 45) blue / (6.76, 88.6) red.
    // Last (optional) number = flywheel velocity added to SHOOTER_FIXED_VELOCITY for that route (R1/B1: +20).
    R1(  54,   7,   90,              60,   7,   90,              6.76, 88.6, 90,   20),   // red, audience wall near hive
    R2(   9,   9,   90,               36,  36,  41.5,               14,  36,   0),   // red, audience-wall corner
    R3(  36, 135,  -90,               36,  36,  41.5,               14,  36,   0),   // red, far wall left of hive
    B1(  90, 137,  270,              84, 137,  270,               137,  45, 270,   20),   // blue, audience wall near hive
    B2( 135, 135,  270,              108, 108, 221.5,              130, 108, 180),   // blue, audience-wall corner
    B3( 108,   9,   90,              108, 108, 221.5,              130, 108, 180);   // blue, far wall right of hive

    /**
     * Added to every shoot heading, in degrees. 0 = the robot's front (+x axis) points at the raised cell,
     * which assumes the shooter faces the front. If the shooter faces another way, put the offset here.
     */
    public static final double SHOOT_HEADING_OFFSET_DEG = 0;

    public final Pose start, shoot, park;
    /** Added to SHOOTER_FIXED_VELOCITY for this route only (the teleop is not affected). */
    public final double velocityBoost;

    BioBuzzAutoRoute(double startX, double startY, double startHeadingDeg,
                     double shootX, double shootY, double shootHeadingDeg,
                     double parkX, double parkY, double parkHeadingDeg) {
        this(startX, startY, startHeadingDeg, shootX, shootY, shootHeadingDeg, parkX, parkY, parkHeadingDeg, 0);
    }

    BioBuzzAutoRoute(double startX, double startY, double startHeadingDeg,
                     double shootX, double shootY, double shootHeadingDeg,
                     double parkX, double parkY, double parkHeadingDeg,
                     double velocityBoost) {
        this.start = new Pose(startX, startY, Math.toRadians(startHeadingDeg));
        this.shoot = new Pose(shootX, shootY, Math.toRadians(shootHeadingDeg + SHOOT_HEADING_OFFSET_DEG));
        this.park  = new Pose(parkX, parkY, Math.toRadians(parkHeadingDeg));
        this.velocityBoost = velocityBoost;
    }
}
