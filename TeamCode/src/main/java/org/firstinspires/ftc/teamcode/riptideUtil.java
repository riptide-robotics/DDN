package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class riptideUtil {

    /**
     * Table of contents:
     * 1. General constants (things that are used gneerally, not specific locations)
     * 3. Autonomous Constants
     */

    /** General constants */
    // COLOR SENSING
    // Green
    public static final float GREEN_R = 0.136f;
    public static final float GREEN_R_STDEV = 0.05f;
    public static final float GREEN_G = 0.578f;
    public static final float GREEN_G_STDEV = 0.15f;
    public static final float GREEN_B = 0.404f;
    public static final float GREEN_B_STDEV = 0.13f;
    // Purple
    public static final float PURPLE_R = 0.346f;
    public static final float PURPLE_R_STDEV = 0.02f;
    public static final float PURPLE_G = 0.376f;
    public static final float PURPLE_G_STDEV = 0.03f;
    public static final float PURPLE_B = 0.686f;
    public static final float PURPLE_B_STDEV = 0.04f;

    //flywheel

    public static double TOP_FLYWHEEL_KP = 0.001;
    public static double BOTTOM_FLYWHEEL_KP = 0.001;

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



// -----------------------------------------------------------------------------
//                          MEET 0 STUFF
// -----------------------------------------------------------------------------
    public static double LONG_DIST_TOP = 4500; // UNDETERMINED
    public static double LONG_DIST_BOT = 4500; // UNDETERMINED

    public static double MID_DIST_TOP = 3500; // UNDETERMINED
    public static double MID_DIST_BOT = 3500; // UNDETERMINED

    public static double SHORT_DIST_TOP = 2500; // UNDETERMINED
    public static double SHORT_DIST_BOT = 2500; // UNDETERMINED

    public static double TRANSFER_POWER = 0.8; // UNDETERMINED


}
