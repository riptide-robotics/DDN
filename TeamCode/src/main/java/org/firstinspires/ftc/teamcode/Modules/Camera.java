package org.firstinspires.ftc.teamcode.Modules;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

// vision stuff
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.riptideUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

/*
 * We have to have two classes because java does not support multiple inheritance
 *
 * This class is the hardware aspect of the camera, the camera class will be used for the pipeline (what processes the feed)
 *
 */

public class Camera {

    ///////////////////////////////////////////////
    ////                                     /////
    ////              VARIABLES              /////
    ////                                     /////
    //////////////////////////////////////////////

    // this used to be called webcamdude, I'm looking at you Aaron (actually Aroon)
    OpenCvWebcam webcam = null;

    AprilTagProcessor tag_processor;
    VisionPortal vision_portal;
    List<AprilTagDetection> detections;

    ///////////////////////////////////////////////
    ////                                     /////
    ////              FUNCTIONS              /////
    ////                                     /////
    //////////////////////////////////////////////

    public Camera(CameraName cameraname) {

        tag_processor = new AprilTagProcessor.Builder()
                .setTagLibrary(riptideUtil.getLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .build();
        // can also be 640 and 488
        // there is also YUY2
        vision_portal = new VisionPortal.Builder()
                .addProcessor(tag_processor)
                //.addProcessor(new CameraPipeline(0.047, 578.272, 578.272, 402.145, 221.506, hardwareMap.get(WebcamName.class, "Webcam 1")))
                .setCamera(cameraname)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                //.enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();
    }

    public List<AprilTagDetection> getDetections() {
        detections = tag_processor.getDetections();
        detections.removeIf(detection -> System.nanoTime() - detection.frameAcquisitionNanoTime > riptideUtil.DETECTION_TIMEOUT);
        return detections;
    }

    public void stop() {
        vision_portal.close();
    }

    public void stop_streaming() {
        vision_portal.stopStreaming();
    }

    public void resume_streaming() {
        vision_portal.resumeStreaming();
    }
}
