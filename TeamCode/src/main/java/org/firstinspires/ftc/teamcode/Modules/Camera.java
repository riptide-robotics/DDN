package org.firstinspires.ftc.teamcode.Modules;

// --- CONSTANTS & OTHER STUFF --- //
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

import android.annotation.SuppressLint;
import android.graphics.Color;

import org.firstinspires.ftc.vision.opencv.Circle;

// --- LISTS --- //
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;


import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
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
    ColorBlobLocatorProcessor blob_processor_purple;
    ColorBlobLocatorProcessor blob_processor_green;
    VisionPortal vision_portal;
    ArrayList<AprilTagDetection> detections;
    ArrayList<ArrayList<Double>> blobs = new ArrayList<>();
    String goalTag;
    CameraName cameraname;
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

    public Camera(HardwareMap hardwareMap) {
        cameraname = hardwareMap.get(WebcamName.class, "Webcam 1");
        tag_processor = new AprilTagProcessor.Builder()
                .setTagLibrary(riptideUtil.getLibrary())
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                //.setCameraPose(cameraPosition, cameraOrientation)
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
                .setCircleFitColor(Color.rgb(0, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();

        // can also be 640 and 488
        // there is also YUY2
        vision_portal = new VisionPortal.Builder()
                .addProcessors(tag_processor, blob_processor_purple, blob_processor_green)
                //.addProcessor(new CameraPipeline(0.047, 578.272, 578.272, 402.145, 221.506, hardwareMap.get(WebcamName.class, "Webcam 1")))
                .setCamera(cameraname)
                .setCameraResolution(new Size(riptideUtil.CAMERA_WIDTH, riptideUtil.CAMERA_HEIGHT))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                //.enableCameraMonitoring(true)
                .setAutoStopLiveView(true)
                .build();
    }

    public void setGoalTag(String goalTag) {
        this.goalTag = goalTag;
    }

    public String getGoalTag() {
        return goalTag;
    }

    public ArrayList<AprilTagDetection> getTagDetections() {
        detections = tag_processor.getDetections();
        detections.removeIf(detection -> System.nanoTime() - detection.frameAcquisitionNanoTime > riptideUtil.DETECTION_TIMEOUT);
        return detections;
    }

    public ArrayList<ArrayList<Double>> getBlobDetections() {
        /*
         * Returns an ArrayList of ArrayLists with 5 elements describing the artifact
         * They are (in order):
         *  - x location (on the camera screen)
         *  - y location (on the camera screen)
         *  - distance away from the camera (needs tuning)
         *  - contour area
         *  - circularity
         */
        blobs.clear();
        List<ColorBlobLocatorProcessor.Blob> blobs_detected = blob_processor_purple.getBlobs();
        blobs_detected.addAll(blob_processor_green.getBlobs());

        ColorBlobLocatorProcessor.Util.filterByCriteria(
                ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                250,
                9000,
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
            double distance = (riptideUtil.LENS_FOCAL_LEN_INCHES * riptideUtil.ARTIFACT_SIZE_INCHES * 480)
                    / (circle_fit.getRadius() * 2 * riptideUtil.SENSOR_HEIGHT);
            ArrayList<Double> temp = new ArrayList<>(Arrays.asList((double) circle_fit.getX(), (double) circle_fit.getY(), distance, (double) blob.getContourArea(), blob.getCircularity()));
            blobs.add(temp);
        }

//        if(blobs_detected.isEmpty()) {return new ArrayList<>();}
//        Circle circle_fit = blobs_detected.get(0).getCircle();
//        blobs.add(new ArrayList<>(Arrays.asList((double) circle_fit.getX(), (double) circle_fit.getY() /*blob.getCircularity(), distance*/)));

        return blobs;
    }

    public AprilTagDetection getGoalApriltag() {
        detections = getTagDetections();
        for(AprilTagDetection detection : detections) {
            if(detection.metadata.name.equals(goalTag)) {
                return detection;
            }
        }
        return null;
    }

    public EditablePose2D getGoalApriltagLocation() {
        detections = getTagDetections();
        for(AprilTagDetection detection : detections) {
            if (!detection.metadata.name.contains("Obelisk")) {
                return new EditablePose2D(
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y, 0,
                        DistanceUnit.INCH);
            }
        }
        return null;
    }

    public double getAprilTagDistance(AprilTagDetection tag) {
        return Math.pow(tag.robotPose.getPosition().x, 2) *
               Math.pow(tag.robotPose.getPosition().y, 2) *
               Math.pow(tag.robotPose.getPosition().z, 2);
    }

    public double getTagHorizontalAngle(AprilTagDetection tag) {
        /*
         * Gets the angle of the april tag relative to the camera.
         */

        // essentially setting the origin to the center of the screen
        double delta_x = (double) riptideUtil.CAMERA_WIDTH / 2 - tag.center.x;

        // relative to the camera
        double horizontal_angle = delta_x * riptideUtil.CAM_FOV / riptideUtil.CAMERA_WIDTH;

        //the angle of the april tag relative to the camera
        return horizontal_angle;
    }

    public EditablePose2D findNearestArtifact() {
        return nearestArtifact(0);
    }

    public EditablePose2D findNearestArtifact(double turret_angle) {
        return nearestArtifact(turret_angle);
    }

    private EditablePose2D nearestArtifact(double turret_angle) {
        /*
         * This is just so the code looks nice and if there is no turret_angle, it is:
         *    findNearestArtifact();
         * instead of:
         *    findNearestArtifact(0);
         * This is litterly a minor, insignificant, useless detail and Aaron will probably yell
         * at me for this and I will probably change this in the future
         * Future Aaron, I predicted your response, and I chose to do this useless thing.
         * hahhahahahahhhahahahahahhahahahahahhahahahahahhashashahahahahahahahahahahahahah
         * ahahahahahahahahahahahahahahahhahahahahahahahahahhahahahahahahahahahahahahahahah
         * ahahhahahahahahhahahhhahahahahahahah
         */
        if(blobs.isEmpty()) {return null;}
        ArrayList<Double> largest_contour = new ArrayList<>(blobs.get(0));

        double distance = largest_contour.get(2);

        // essentially setting the origin to the center of the screen
        double delta_x = (double) riptideUtil.CAMERA_WIDTH / 2 - largest_contour.get(0);
        double delta_y = (double) riptideUtil.CAMERA_HEIGHT / 2 - largest_contour.get(1);

        // relative to the camera
        double horizontal_distance = delta_x * riptideUtil.CAM_FOV / riptideUtil.CAMERA_WIDTH;
        double vertical_distance = delta_y * riptideUtil.CAM_FOV / riptideUtil.CAMERA_HEIGHT;

        double horizontal_angle_error = Math.atan(horizontal_distance/distance);
        double vertical_angle_error = Math.atan(vertical_distance/distance);

        // difference between the errors and the camera angle, so this is relative to what ever
        // the angle of the camera is relative to
        double horizontal_angle = turret_angle - horizontal_angle_error;
        double vertical_angle = riptideUtil.CAMERA_ANGLE - vertical_angle_error;

        // relative to whatever the turret_angle is
        // 0 degrees means relative to the camera or robot (if they are looking in the same direction)
        // anything else would imply relative to the robot
        double horizontal_dist = Math.sin(horizontal_angle_error) * distance;

        // relative to the ground
        double height = Math.sin(vertical_angle) * distance;

        return new EditablePose2D(
                horizontal_dist,
                height,
                0,
                DistanceUnit.INCH
        );
    }

    // the default is ALL
    public void setPipeline(processors_enabled processors) {
        switch (processors) {
            case TAG:
                vision_portal.setProcessorEnabled(tag_processor, true);
                vision_portal.setProcessorEnabled(blob_processor_purple, false);
                vision_portal.setProcessorEnabled(blob_processor_green, false);
                break;
            case COLOR:
                vision_portal.setProcessorEnabled(tag_processor, false);
                vision_portal.setProcessorEnabled(blob_processor_purple, true);
                vision_portal.setProcessorEnabled(blob_processor_green, true);
                break;
            case NONE:
                vision_portal.setProcessorEnabled(tag_processor, false);
                vision_portal.setProcessorEnabled(blob_processor_purple, false);
                vision_portal.setProcessorEnabled(blob_processor_green, false);
                break;
            default:
                vision_portal.setProcessorEnabled(tag_processor, true);
                vision_portal.setProcessorEnabled(blob_processor_purple, true);
                vision_portal.setProcessorEnabled(blob_processor_green, true);
        }
    }

    public void stopStreaming() {
        vision_portal.stopStreaming();
    }

    public void resumeStreaming() {
        vision_portal.resumeStreaming();
    }

    public void stop() {
        vision_portal.close();
    }

    @SuppressLint("DefaultLocale")
    public void distanceFromGoal(double x, double y, double z){

        //telemetry.addLine(String.format(" --- %d AprilTags Detected --- ", detections.size()));
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                //telemetry.addLine(String.format("%s (ID %d)", detection.metadata.name, detection.id));
                if (!detection.metadata.name.contains("Obelisk")) {
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getPosition().z));
                    telemetry.addLine(String.format("Distance %f (inch)",
                            getAprilTagDistance(detection)));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                }
            } else {
                telemetry.addLine(String.format("Unknown Name (ID %d)", detection.id));
            }


            telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));
            x = detection.robotPose.getPosition().x;
            y = detection.robotPose.getPosition().y;
            z = detection.robotPose.getPosition().z;
        }

        telemetry.addLine(String.format(" --- %d Artifacts Detected --- ", blobs.size()));

        for (List<Double> blob : blobs) {
            telemetry.addLine(String.format("Position: (%f, %f)", blob.get(0), blob.get(1)));
            telemetry.addLine(String.format("Circularity: %f", blob.get(4)));
            telemetry.addLine(String.format("Contour Area: %f", blob.get(3)));
            telemetry.addLine(String.format("Distance: %f Inches Away", blob.get(2)));
        }
    }

    public void tagAngle(double angle){

    }

    public Double getGoalDistance() {
        AprilTagDetection goalDetection = getGoalApriltag();
        if (goalDetection == null) {
            return null;
        }

        double x = goalDetection.robotPose.getPosition().x;
        double y = goalDetection.robotPose.getPosition().y;
        double z = goalDetection.robotPose.getPosition().z;

        return Math.sqrt(x * x + y * y + z * z);
    }

    public Double getGoalAngleError() {
        AprilTagDetection goalDetection = getGoalApriltag();
        if (goalDetection == null) {
            return null;
        }

        return getTagHorizontalAngle(goalDetection);
    }
}
