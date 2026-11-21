package org.firstinspires.ftc.teamcode.biobuzz.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.biobuzz.shooter.PIDFShooter;

/**
 * The two BioBuzz teleops, Blue and Red. They differ only in the alliance; the logic is in
 * {@link BioBuzzTeleOpBase}. The flywheel is driven by the hub velocity PIDF (SHOOTER_P / I / D / F in BioBuzzConfig).
 */
public final class BioBuzzTeleOps {
    private BioBuzzTeleOps() {}

    @TeleOp(name = "BioBuzz TeleOp Blue", group = "BioBuzz")
    public static class PIDFBlue extends BioBuzzTeleOpBase {
        public PIDFBlue() { super(BioBuzzAlliance.BLUE, new PIDFShooter()); }
    }

    @TeleOp(name = "BioBuzz TeleOp Red", group = "BioBuzz")
    public static class PIDFRed extends BioBuzzTeleOpBase {
        public PIDFRed() { super(BioBuzzAlliance.RED, new PIDFShooter()); }
    }
}
