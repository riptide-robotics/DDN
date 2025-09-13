package org.firstinspires.ftc.teamcode.Tuning;

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
    @Override
    public void runOpMode() throws InterruptedException {

        Camera camera = new Camera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        waitForStart();
        while (!isStopRequested() && opModeIsActive()) {
            List<AprilTagDetection> detections = camera.getDetections();
            if(!detections.isEmpty()) {
                AprilTagDetection tag = detections.get(0);
                telemetry.addData("ID: ", tag.metadata.id);
                telemetry.addData("Name: ", tag.metadata.name);
            }
            telemetry.update();
        }
    }
}
