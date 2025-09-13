package org.firstinspires.ftc.teamcode.Tuning;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Modules.AprilTagCustomDatabase;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
public class TestVision extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        AprilTagProcessor tag_processor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagCustomDatabase.getLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();

        // can also be 640 and 488
        // there is also YUY2
        VisionPortal vision_portal = new VisionPortal.Builder()
                .addProcessor(tag_processor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            if (!tag_processor.getDetections().isEmpty()) {
                AprilTagDetection tag = tag_processor.getDetections().get(0);
            }
        }
    }
}
