package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@Config
public class riptideUtil {

    /**
     * Table of contents:
     * 1. General constants (things that are used gneerally, not specific locations)
     * 3. Autonomous Constants
     */

    /** General constants */


    /** Autonomous Constants */
    public static final double POINT_TOLERANCE = 2; // UNDETERMINED
    // Maximums
    public static double MAX_A_VERT = 72;            // UNDETERMINED
    public static double MAX_V_VERT = 96;            // UNDETERMINED
    public static double MAX_A_LAT = 0;             // UNDETERMINED
    public static double MAX_V_LAT = 0;             // UNDETERMINED

    public static double MAX_WHEEL_POWER = 1;       // Probably always going to be 1
    // Lateral
    public static double LAT_KP = 0.1;                // UNDETERMINED
    public static double LAT_KI = 0.05;                // UNDETERMINED
    public static double LAT_KD = 0;                // UNDETERMINED
    // Vertical
    public static double VERT_KP = 0.065;               // UNDETERMINED
    public static double VERT_KI = 0.03;               // UNDETERMINED
    public static double VERT_KD = 0.003;               // UNDETERMINED
    // Turn
    public static double TURN_KP = 0.03;               // UNDETERMINED
    public static double TURN_KI = 0;               // UNDETERMINED
    public static double TURN_KD = 0.002;               // UNDETERMINED

    // Speed relationships
    public static double MAX_A = 72;                 // UNDETERMINED
    public static double MAX_V = 96;                 // UNDETERMINED

    // ----- VISION ----- //

    public static double DETECTION_TIMEOUT = 25e+7;

    public static double ARTIFACT_SIZE_INCHES = 5;
    public static double LENS_FOCAL_LEN_INCHES = 0.15748;
    public static double LENS_HEIGHT_OFF_GROUND_INCHES = 2;

    public static AprilTagLibrary getLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(
                        20,
                        "Blue Goal",
                        6.5,
                        //new VectorF(),
                        DistanceUnit.INCH
                        //Quaternion.identityQuaternion()
                )
                .addTag(
                        21,
                        "Green Purple Purple (GPP)",
                        6.5,
                        DistanceUnit.INCH
                )
                .addTag(
                        22,
                        "Purple Green Purple (PGP)",
                        6.5,
                        DistanceUnit.INCH
                )
                .addTag(
                        23,
                        "Purple Purple Green (PPG)",
                        6.5,
                        DistanceUnit.INCH
                )
                .addTag(
                        24,
                        "Red Goal",
                        6.5,
                        //new VectorF(),
                        DistanceUnit.INCH
                        //Quaternion.identityQuaternion()
                )
                .build();
    }
}
