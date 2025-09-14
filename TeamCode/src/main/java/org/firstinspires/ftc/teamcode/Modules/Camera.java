package org.firstinspires.ftc.teamcode.Modules;

// --- CONSTANTS & OTHER STUFF --- //
import org.firstinspires.ftc.teamcode.riptideUtil;

// --- CAMERA --- //
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.openftc.easyopencv.OpenCvWebcam;

// --- PORTALS & PROCESSORS & SIMILAR STUFF --- //
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.Circle;

// --- LISTS --- //
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import android.util.Size;

import com.qualcomm.robotcore.util.SortOrder;

/*
    Resources:
        - https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/master/FtcRobotController/src/main/java/org/firstinspires/ftc/robotcontroller/external/samples/ConceptVisionColorLocator_Rectangle.java
        -
 */

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
    ColorBlobLocatorProcessor blob_processor;
    VisionPortal vision_portal;
    List<AprilTagDetection> detections;
    List<List<Double>> blobs;

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

        blob_processor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setDrawContours(true)
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // can also be 640 and 488
        // there is also YUY2
        vision_portal = new VisionPortal.Builder()
                .addProcessors(tag_processor, blob_processor)
                //.addProcessor(new CameraPipeline(0.047, 578.272, 578.272, 402.145, 221.506, hardwareMap.get(WebcamName.class, "Webcam 1")))
                .setCamera(cameraname)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                //.enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();
    }

    public List<AprilTagDetection> get_tag_detections() {
        detections = tag_processor.getDetections();
        detections.removeIf(detection -> System.nanoTime() - detection.frameAcquisitionNanoTime > riptideUtil.DETECTION_TIMEOUT);
        return detections;
    }

    public List<List<Double>> get_blob_detections() {
        List<ColorBlobLocatorProcessor.Blob> blobs_detected = blob_processor.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                50,
                2000,
                blobs_detected
        );

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.75,
                1,
                blobs_detected
        );

        ColorBlobLocatorProcessor.Util.sortByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                SortOrder.DESCENDING,
                blobs_detected
        );

        for (ColorBlobLocatorProcessor.Blob blob : blobs_detected) {
            // the circle which fits the artifacts
            Circle circle_fit = blob.getCircle();
            double distance = (riptideUtil.LENS_FOCAL_LEN_INCHES * riptideUtil.ARTIFACT_SIZE_INCHES * 480)
                    / (circle_fit.getRadius() * 2 * riptideUtil.LENS_HEIGHT_OFF_GROUND_INCHES);
            blobs.add(Arrays.asList((double) circle_fit.getX(), (double) circle_fit.getY(), distance));
        }
        return blobs;
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
