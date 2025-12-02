package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

@Config
public class riptideUtil {

    /**
     * Table of contents:
     * 1. General constants (things that are used gneerally, not specific locations)
     * 2. Autonomous Constants
     */

    /** General constants */
    // COLOR SENSING
    // Green
    public static final float GREEN_R = 0.15f;
    public static final float GREEN_R_STDEV = 0.04f;
    public static final float GREEN_G = 0.605f;
    public static final float GREEN_G_STDEV = 0.175f;
    public static final float GREEN_B = 0.45f;
    public static final float GREEN_B_STDEV = 0.127f;

    public static final float GREEN_R_HOLE = 0.019f;
    public static final float GREEN_R_STDEV_HOLE = 0.0045f;
    public static final float GREEN_G_HOLE = 0.056f;
    public static final float GREEN_G_STDEV_HOLE = 0.018f;
    public static final float GREEN_B_HOLE = 0.045f;
    public static final float GREEN_B_STDEV_HOLE = 0.011f;

    // Purple
    public static final float PURPLE_R = 0.239f;
    public static final float PURPLE_R_STDEV = 0.09f;
    public static final float PURPLE_G = 0.262f;
    public static final float PURPLE_G_STDEV = 0.1f;
    public static final float PURPLE_B = 0.48f;
    public static final float PURPLE_B_STDEV = 0.19f;

    public static final float PURPLE_R_HOLE = 0.034f;
    public static final float PURPLE_R_STDEV_HOLE = 0.023f;
    public static final float PURPLE_G_HOLE = 0.053f;
    public static final float PURPLE_G_STDEV_HOLE = 0.027f;
    public static final float PURPLE_B_HOLE = 0.089f;
    public static final float PURPLE_B_STDEV_HOLE = 0.05f;

    //flywheel
    public static double TOP_FLYWHEEL_KP = 0.001;
    public static double BOTTOM_FLYWHEEL_KP = 0.001;

    // Turntable
    public static final double TICKS_TO_DEGREES = 360/751.8;
    public static final double DEGREES_TO_TICKS = 751.8/360;
    public static final double DEADZONE = 3;

    public static final double TURNTABLE_KP = 0.002;
    public static final double TURNTABLE_KI = 0.003;
    public static final double TURNTABLE_KD = 0.00005;

    /** Autonomous Constants */

    public static final Pose2D START_POSITION = new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90);

    public static final double POINT_TOLERANCE = 2; // UNDETERMINED
    // Maximums
    public static double MAX_V_TURN = 90;   // deg/s //UNDETERMINED

    public static double MAX_A_VERT = 72;            // UNDETERMINED
    public static double MAX_V_VERT = 96;            // UNDETERMINED

    public static double MAX_WHEEL_POWER = 1;       // Probably always going to be 1

    // Forward
    public static double FORWARD_KP = 0.065;               // UNDETERMINED
    public static double FORWARD_KI = 0.03;               // UNDETERMINED
    public static double FORWARD_KD = 0.003;               // UNDETERMINED
    // Turn
    public static double TURN_KP = 0.03;
    public static double TURN_KI = 0.02;
    public static double TURN_KD = 0.0015;

    // Speed relationships
    public static double MAX_A = 72;                 // UNDETERMINED
    public static double MAX_V = 96;                 // UNDETERMINED


    public static double LONG_DIST_TOP = 3250; // UNDETERMINED
    public static double LONG_DIST_BOT = 3730; // UNDETERMINED

    public static double MID_DIST_TOP = 2650; // UNDETERMINED
    public static double MID_DIST_BOT = 3000;// UNDETERMINED

    public static double SHORT_DIST_TOP = 2950; // UNDETERMINED
    public static double SHORT_DIST_BOT = 3480;// UNDETERMINED

    public static double KPTop = 0.0032; //0.004
    public static double KPBottom = 0.004; // 0.0045

    public static double tolerance = 50; //?? what

    // Spindex
    public static double SPINDEX_ARM_UP = 0.5; // UNDETERMINED
    public static double SPINDEX_ARM_RESTING = 0.1; // UNDETERMINED
    public static double ROTATE_SPINDEX_ONCE = 60;

    // POS 1
    // 0 shoot
    // 180 pickup

    // POS 2
    // 120shoot
    // 300 pickup

    // POS 3
    // 240 shoot
    // 60 pickup

    public static double SLOT_ONE_SHOOT_POS = 0;
    public static double SLOT_ONE_PIKCUP_POS = 180;

    public static double SLOT_TWO_SHOOT_POS = 120;
    public static double SLOT_TWO_PIKCUP_POS = 300;

    public static double SLOT_THREE_SHOOT_POS = 240;
    public static double SLOT_THREE_PIKCUP_POS = 60;

    public static double SPINDEX_SPEED = 0.03;

    // Vision
    public static int CAMERA_WIDTH = 640;
    public static int CAMERA_HEIGHT = 480;
    public static double CAMERA_ANGLE = 0;

    public static double DETECTION_TIMEOUT = 25e+7;
    public static double ARTIFACT_SIZE_INCHES = 5;

    public static double LENS_FOCAL_LEN_INCHES = 0.15748;
    public static double SENSOR_HEIGHT = 0.086;
    public static double CAM_FOV = 55;

    public static double MOTOR_POS_CONST = 1; // needs tuning

    public static enum TEAM_COLOR {
        RED,
        BLUE
    }

    public static AprilTagLibrary getLibrary() {
        return new AprilTagLibrary.Builder()
                .addTag(
                        20,
                        "Blue Goal",
                        6.5,
                        //new VectorF(-0f, -0f, 0f),
                        DistanceUnit.INCH//,
                        //Quaternion.identityQuaternion()
                )
                .addTag(
                        21,
                        "Obelisk Green Purple Purple (GPP)",
                        6.5,
                        DistanceUnit.INCH
                )
                .addTag(
                        22,
                        "Obelisk Purple Green Purple (PGP)",
                        6.5,
                        DistanceUnit.INCH
                )
                .addTag(
                        23,
                        "Obelisk Purple Purple Green (PPG)",
                        6.5,
                        DistanceUnit.INCH
                )
                .addTag(
                        24,
                        "Red Goal",
                        6.5,
                        //new VectorF(-0f, -0f, -0f),
                        DistanceUnit.INCH//,
                        //Quaternion.identityQuaternion()
                )
                .build();
    }

    public static double angularVelocity = 45;
    public static double econserved = 0.1;
}
