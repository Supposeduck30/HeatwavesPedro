package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.shooter.PIDFShooter;

/**
 * Experiment teleops: the normal PIDF teleop ({@link BioBuzzTeleOpBase}) but with its OWN P / I / D / F, so you
 * can try tuning ideas without touching BioBuzzConfig or the real PIDF / Bang Bang teleops. Change the four
 * numbers below, redeploy, drive. Everything else (controls, shooter speed, F voltage compensation, feed power)
 * is the same as "BioBuzz TeleOp Blue / Red".
 *
 * When a combination works, copy it into SHOOTER_P / SHOOTER_I / SHOOTER_D / SHOOTER_F in BioBuzzConfig.
 *
 * Experiment log (newest first):
 *   target 1400, P = 80, F = 13.07, intake/transfer 1.0   (config at the time: target 1480, P = 67, F = 13.303, intake/transfer 0.85)
 *   target 1400, P = 100, F = 15, intake/transfer 0.95
 *   P = 80, F = 15, intake/transfer 0.95, target 1480
 */
public final class ExperimentTeleOps {
    private ExperimentTeleOps() {}

    // ================= EXPERIMENT (change these) =================
    /** Shooter target in ticks/s (the normal teleops use SHOOTER_FIXED_VELOCITY = 1460). */
    public static final double EXP_TARGET_VELOCITY = 1460;
    public static final double EXP_P = 130; // 2026-11-20 (was 135, 80)
    public static final double EXP_I = 0;
    public static final double EXP_D = 0;
    /** Used as-is, like SHOOTER_F (no battery-voltage scaling). */
    public static final double EXP_F = 13.85; // 2026-11-20 (was 14.6, 14.664, 12.627)
    /** Intake AND transfer power on gamepad2 right bumper (the normal PIDF teleop uses 0.85). Full speed here. */
    public static final double EXP_INTAKE_TRANSFER_POWER = 1.0;

    @TeleOp(name = "Experiment TeleOp Blue", group = "BioBuzz")
    public static class ExperimentBlue extends BioBuzzTeleOpBase {
        public ExperimentBlue() {
            super(BioBuzzAlliance.BLUE, new PIDFShooter(EXP_P, EXP_I, EXP_D, EXP_F, EXP_INTAKE_TRANSFER_POWER));
        }
        @Override protected double fixedShooterVelocity() { return EXP_TARGET_VELOCITY; }
    }

    @TeleOp(name = "Experiment TeleOp Red", group = "BioBuzz")
    public static class ExperimentRed extends BioBuzzTeleOpBase {
        public ExperimentRed() {
            super(BioBuzzAlliance.RED, new PIDFShooter(EXP_P, EXP_I, EXP_D, EXP_F, EXP_INTAKE_TRANSFER_POWER));
        }
        @Override protected double fixedShooterVelocity() { return EXP_TARGET_VELOCITY; }
    }
}
