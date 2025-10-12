package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_B;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_B_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_G;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_G_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_R;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_R_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_B;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_B_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_G;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_G_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_R;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_R_STDEV;

import java.lang.Override;
// Imports to sync

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot;

public class Intake{
    String[] order = new String[3];
    TelemetryPacket t = new TelemetryPacket();
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


    public Intake(HardwareMap hardwareMap){

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "REVcolorSensor");
    }

    public String scanColor() {
        NormalizedRGBA detectedColor = colorSensor.getNormalizedColors();

        //return "Red: " + detectedColor.red * 10 + " - Green: " + detectedColor.green * 10 + " - Blue: " + detectedColor.blue * 10 + " - Alpha: " + detectedColor.alpha;

        if ((detectedColor.red < PURPLE_R + PURPLE_R_STDEV
                    &&
             detectedColor.red > PURPLE_R - PURPLE_R_STDEV)
                &&
            (detectedColor.green < PURPLE_G + PURPLE_G_STDEV
                    &&
             detectedColor.green > PURPLE_G - PURPLE_G_STDEV)
                &&
            (detectedColor.blue < PURPLE_B + PURPLE_B_STDEV
                    &&
             detectedColor.blue > PURPLE_B - PURPLE_B_STDEV)) {
            return "Purple";
        } else if ((detectedColor.red < GREEN_R + GREEN_R_STDEV
                            &&
                    detectedColor.red > GREEN_R - GREEN_R_STDEV)
                        &&
                   (detectedColor.green < GREEN_G + GREEN_G_STDEV
                            &&
                    detectedColor.green > GREEN_G - GREEN_G_STDEV)
                        &&
                   (detectedColor.blue < GREEN_B + GREEN_B_STDEV
                            &&
                    detectedColor.blue > GREEN_B - GREEN_B_STDEV)) {
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
            t.addLine("Uptake result: Failure - Full Intake");
            return;
        }

        String color = scanColor();
        if (color.equals("Purple")) {
            pgratio[0]++;
        } else if (color.equals("Green")) {
            pgratio[1]++;
        }
        order[toSet] = color;
        t.addLine("Uptake Result: Success - " + color + " artifact uptaked into position " + toSet);

        // UPTAKE FUNCTION
    }

    public void ejectArtifact(char colorReq){ // colorReq should b 'p' (purple) or 'g' (green)
        String tmp;
        if (order[0] == null || order[0].toLowerCase().charAt(0) != colorReq) {
            if (order[1] == null || order[1].toLowerCase().charAt(0) != colorReq) {
                if (order[2] == null || order[2].toLowerCase().charAt(0) != colorReq) {
                    t.addLine("Eject result: Failed search");
                    t.addLine("Action: Ejecting artifact");
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
                    t.addLine("Eject result: Target found at pos 2");
                    rotateOne(false);
                    // ROTATOR FUNCTION
                    t.addLine("Eject status: Rotated -1 units");
                    t.addLine("Action: Ejecting artifact");
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
                t.addLine("Eject result: Target found at pos 1");
                rotateOne(true);
                // ROTATOR FUNCTION W/IN ROTATE
                t.addLine("Eject status: Rotated 1 units");
                t.addLine("Action: Ejecting artifact");
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
            t.addLine("Eject result: Target found at pos 0");
            t.addLine("Eject status: At desired position");
            t.addLine("Action: Ejecting artifact");
            // EJECT FUNCTION
            tmp = order[0];
            order[0] = null;
            if (tmp.toLowerCase().charAt(0) == 'p') {
                pgratio[0]--;
            } else {
                pgratio[1]--;
            }
        }
        rotateOne(true);
        // Depending on how the intake storage is built, may need to ROTATE 1 unit
        t.addLine("P:G - " + pgratio[0] + ":" + pgratio[1]);
    }

    public void rotateOne(boolean forwards) { // HAVE ROTATOR FUNCTIONALITY W/IN THIS FUNCTION
        String tmp = order[0];
        if (forwards) {
            order[0] = order[1];
            order[1] = order[2];
            order[2] = tmp;
        } else {
            order[0] = order[2];
            order[2] = order[1];
            order[1] = tmp;
        }
    }

    public TelemetryPacket sendTelemetry(){
        return t;
    }
}
