package org.firstinspires.ftc.teamcode.Modules;

import java.lang.Override;
// Imports to sync

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public class Intake extends LinearOpMode {
    Robot robot;
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    String[] order = new String[3];
    int[] pgratio = new int[2];

    NormalizedColorSensor colorSensor;
    float gain = 2; // ...... ANYWAY
    /**
     * A Rev Color Match object is used to register and detect known colors. This can
     * be calibrated ahead of time or during operation.
     *
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */

    /**
     * These colors must be recalibrated to represent purple & green
     */
    private final NormalizedRGBA purple = new NormalizedRGBA();
    private final NormalizedRGBA green = new NormalizedRGBA();

    @Override
    public void runOpMode() throws InterruptedException {
        purple.red = 0.68f;
        purple.green = 0.29f;
        purple.blue = 0.72f;
        purple.alpha = 0.95f;
        green.red = 0.17f;
        green.green = 0.73f;
        green.blue = 0.49f;
        green.alpha = 0.95f;
        // THIS PART NEEDS TO BE RETUNED :')


        robot = new Robot(hardwareMap);
        colorSensor = robot.getColorSensor();

        /**
         * The method GetColor() returns a normalized color value from the sensor and can be
         * useful if outputting the color to an RGB LED or similar. To
         * read the raw color, use GetRawColor().
         *
         * The color sensor works best when within a few inches from an object in
         * well lit conditions (the built in LED is a big help here!). The farther
         * an object is the more light from the surroundings will bleed into the
         * measurements and make it difficult to accurately determine its color.
         */

        /**
         * Run the color match algorithm on our detected color
         */
        String color = scanColor();
        if (color.equals("Purple")) {
            t.addData("Color", "Purple");
        } else if (color.equals("Green")) {
            t.addData("Color", "Green");
        } else {
            t.addData("Color", "Not Purple or Green");
        }

        t.update();
    }

    public String scanColor() {
        NormalizedRGBA detectedColor = colorSensor.getNormalizedColors();

        if ((detectedColor.red < purple.red + 0.08
                    &&
             detectedColor.red > purple.red - 0.08)
                &&
            (detectedColor.green < purple.green + 0.08
                    &&
             detectedColor.green > purple.green - 0.08)
                &&
            (detectedColor.blue < purple.blue + 0.08
                    &&
             detectedColor.blue > purple.blue - 0.08)) {
            return "Purple";
        } else if ((detectedColor.red < green.red + 0.08
                            &&
                    detectedColor.red > green.red - 0.08)
                        &&
                   (detectedColor.green < green.green + 0.08
                            &&
                    detectedColor.green > green.green - 0.08)
                        &&
                   (detectedColor.blue < green.blue + 0.08
                            &&
                    detectedColor.blue > green.blue - 0.08)) {
            return "Green";
        } else {
            return "None";
        }
    }

    public void uptakeTarget() {
        int toSet;
        if (order[0] == null) { // EMPTY INTAKE
            toSet = 0;
        } else if (order[1] == null) {
            toSet = 1;
        } else if (order[2] == null) {
            toSet = 2;
        } else { // FULL INTAKE
            toSet = -1;
        }
        if (toSet == -1) {
            t.addData("Uptake Result", "Full Intake");
            t.update();
            return;
        }

        String color = scanColor();
        if (color.equals("Purple")) {
            pgratio[0]++;
        } else if (color.equals("Green")) {
            pgratio[1]++;
        }
        order[toSet] = color;
        t.addData("Uptake Result", color + " artifact uptaked into position " + toSet);
        t.update();

        // UPTAKE FUNCTION
    }

    public void ejectArtifact(char colorReq){ // colorReq should b 'p' (purple) or 'g' (green)
        String tmp;
        if (order[0] == null || order[0].toLowerCase().charAt(0) != colorReq) {
            if (order[1] == null || order[1].toLowerCase().charAt(0) != colorReq) {
                if (order[2] == null || order[2].toLowerCase().charAt(0) != colorReq) {
                    t.addData("Result", "Failed search");
                    t.addData("Action", "Ejecting artifact");
                    tmp = order[0];
                    order[0] = null;
                    if (tmp == null) {
                        pgratio[0]++;
                        pgratio[0]--;
                    } else if (tmp.toLowerCase().charAt(0) == 'p') {
                        pgratio[0]--;
                    } else {
                        pgratio[1]--;
                    }
                } else {
                    t.addData("Result", "Target found at pos 2");
                    tmp = order[2];
                    order[2] = order[1];
                    order[1] = order[0];
                    order[0] = tmp;
                    // ROTATOR FUNCTION
                    t.addData("Status", "Rotated -1 units");
                    t.addData("Action", "Ejecting artifact");
                    // EJECT FUNCTION
                    tmp = order[0];
                    order[0] = null;
                    if (tmp.toLowerCase().charAt(0) == 'p') {
                        pgratio[0]--;
                    } else {
                        pgratio[1]--;
                    }
                }
            } else {
                t.addData("Result", "Target found at pos 1");
                tmp = order[1];
                order[1] = order[2];
                order[2] = order[0];
                order[0] = tmp;
                // ROTATOR FUNCTION
                t.addData("Status", "Rotated 1 units");
                t.addData("Action", "Ejecting artifact");
                // EJECT FUNCTION
                tmp = order[0];
                order[0] = null;
                if (tmp.toLowerCase().charAt(0) == 'p') {
                    pgratio[0]--;
                } else {
                    pgratio[1]--;
                }
            }
        } else {
            t.addData("Result", "Target found at pos 0");
            t.addData("Status", "At desired position");
            t.addData("Action", "Ejecting artifact");
            // EJECT FUNCTION
            tmp = order[0];
            order[0] = null;
            if (tmp.toLowerCase().charAt(0) == 'p') {
                pgratio[0]--;
            } else {
                pgratio[1]--;
            }
        }
        order[0] = order[1];
        order[1] = order[2];
        order[2] = null;
        // Depending on how the intake storage is built, may need to ROTATE 1 unit
        t.addData("", "P:G - " + pgratio[0] + ":" + pgratio[1]);
        t.update();
    }
}
