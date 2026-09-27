package org.firstinspires.ftc.teamcode.biobuzz;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.biobuzz.BallBlobPipeline.BallColor;
import org.firstinspires.ftc.teamcode.biobuzz.BallBlobPipeline.BallBlob;


public class AutoBallDetectionTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private LabBallDetectorPipeline ballDetector;

    @Override
    public void runOpMode() {
        // 1. Create the detector
        ballDetector = new LabBallDetectorPipeline();

        // 2. Start the camera and attach the detector
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(ballDetector)
                .build();

        waitForStart();

        while (opModeIsActive()) {
            // 3. Ask the pipeline for the best yellow ball candidate
            BallBlob bestYellow = ballDetector.bestOf(BallColor.YELLOW);

            if (bestYellow != null) {
                telemetry.addData("Found Yellow!", "X: %.1f, Y: %.1f", bestYellow.cx, bestYellow.cy);
                // TODO: Feed bestYellow.cx into your drive code to steer
            } else {
                telemetry.addLine("No yellow balls in sight.");
            }
            telemetry.update();
        }
    }
}