package org.firstinspires.ftc.teamcode.Modules;

// --- CONSTANTS & OTHER STUFF --- //
import org.firstinspires.ftc.teamcode.riptideUtil;

// --- CAMERA --- //
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.openftc.easyopencv.OpenCvWebcam;

// --- PORTALS & PROCESSORS & SIMILAR STUFF --- //
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import android.graphics.Color;
import org.opencv.core.Scalar;
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
    ArrayList<AprilTagDetection> detections;
    ArrayList<ArrayList<Double>> blobs = new ArrayList<>();
    public enum processors_enabled {
        NONE,
        TAG,
        COLOR,
        ALL
    }

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

        blob_processor_purple = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 0, 255))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        blob_processor_green = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -1))
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 0, 255))
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

    public ArrayList<ArrayList<Double>> get_blob_detections() {
        blobs.clear();
        List<ColorBlobLocatorProcessor.Blob> blobs_detected = blob_processor.getBlobs();

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                10,
                5000,
                blobs_detected
        );

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                0.1,
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
//            double distance = (riptideUtil.LENS_FOCAL_LEN_INCHES * riptideUtil.ARTIFACT_SIZE_INCHES * 480)
//                    / (circle_fit.getRadius() * 2 * riptideUtil.LENS_HEIGHT_OFF_GROUND_INCHES);
            ArrayList<Double> temp = new ArrayList<>(Arrays.asList((double) circle_fit.getX(), (double) circle_fit.getY(), blob.getCircularity(), (double) blob.getContourArea()/*, distance*/));
            blobs.add(temp);
        }

//        if(blobs_detected.isEmpty()) {return new ArrayList<>();}
//        Circle circle_fit = blobs_detected.get(0).getCircle();
//        blobs.add(new ArrayList<>(Arrays.asList((double) circle_fit.getX(), (double) circle_fit.getY() /*blob.getCircularity(), distance*/)));

        return blobs;
    }

    // the default is ALL
    public void set_pipeline(processors_enabled processors) {
        switch (processors) {
            case TAG:
                vision_portal.setProcessorEnabled(tag_processor, true);
                vision_portal.setProcessorEnabled(blob_processor, false);
                break;
            case COLOR:
                vision_portal.setProcessorEnabled(tag_processor, false);
                vision_portal.setProcessorEnabled(blob_processor, true);
                break;
            case NONE:
                vision_portal.setProcessorEnabled(tag_processor, false);
                vision_portal.setProcessorEnabled(blob_processor, false);
                break;
            default:
                vision_portal.setProcessorEnabled(tag_processor, true);
                vision_portal.setProcessorEnabled(blob_processor, true);
        }
    }

    public void stop_streaming() {
        vision_portal.stopStreaming();
    }

    public void resume_streaming() {
        vision_portal.resumeStreaming();
    }

    public void stop() {
        vision_portal.close();
    }
}
