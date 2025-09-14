package org.firstinspires.ftc.teamcode.Tuning;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Modules.AprilTagCustomDatabase;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class TestVision extends LinearOpMode {
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        Camera camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            List<AprilTagDetection> detections = camera.getDetections();

            telemetry.addLine(String.format(" --- %d AprilTags Detected --- ", detections.size()));

            for (AprilTagDetection detection : detections) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("%s (ID %d)", detection.metadata.name, detection.id));
                } else {
                    telemetry.addLine(String.format("Unknown Name (ID %d)", detection.id));
                }
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));

            }

            telemetry.update();
        }

        camera.stop();
    }
}
