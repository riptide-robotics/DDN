package org.firstinspires.ftc.teamcode.Tuning;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="Test Vision")
public class TestVision extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Camera camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        Camera.processors_enabled processor = Camera.processors_enabled.ALL;
        camera.set_pipeline(processor);
        boolean a_debounce = false;

        telemetry.setMsTransmissionInterval(100);   // speed up telemetry updates for debugging
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            // this will switch the viewed pipeline on the driver hub
            // in this order: ALL -> TAG -> COLOR
            if (gamepad1.a) {
                if(gamepad1.a ^ a_debounce) {
                    switch (processor) {
                        case ALL:
                            processor = Camera.processors_enabled.TAG;
                            camera.set_pipeline(processor);
                            break;
                        case TAG:
                            processor = Camera.processors_enabled.COLOR;
                            camera.set_pipeline(processor);
                            break;
                        case COLOR:
                            processor = Camera.processors_enabled.ALL;
                            camera.set_pipeline(processor);
                            break;
                    }
                }
                a_debounce = true;
            } else {
                a_debounce = false;
            }
            telemetryStuff(camera);
        }

        camera.stop();
    }

    @SuppressLint("DefaultLocale")
    public void telemetryStuff(Camera camera) {
        List<AprilTagDetection> detections = camera.get_tag_detections();
        ArrayList<ArrayList<Double>> color_blobs = camera.get_blob_detections();

        telemetry.addLine(String.format(" --- %d AprilTags Detected --- ", detections.size()));

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("%s (ID %d)", detection.metadata.name, detection.id));
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("Distance %f (inch)",
                            Math.sqrt(detection.robotPose.getPosition().x*detection.robotPose.getPosition().x
                            +detection.robotPose.getPosition().y*detection.robotPose.getPosition().y
                            +detection.robotPose.getPosition().z*detection.robotPose.getPosition().z)));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("Unknown Name (ID %d)", detection.id));
            }
            telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));

        }

        telemetry.addLine(String.format(" --- %d Artifacts Detected --- ", color_blobs.size()));

        for (List<Double> blob : color_blobs) {
            telemetry.addLine(String.format("Position: (%f, %f)", blob.get(0), blob.get(1)));
            telemetry.addLine(String.format("Circularity: %f", blob.get(2)));
            telemetry.addLine(String.format("Contour Area: %f", blob.get(3)));
            telemetry.addLine(String.format("Distance: %f Inches Away", blob.get(4)));
        }

        telemetry.update();
        sleep(100);
    }
}
