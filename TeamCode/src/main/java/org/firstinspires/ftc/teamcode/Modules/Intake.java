package org.firstinspires.ftc.teamcode.Modules;

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
    Robot robot;
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
    private final NormalizedRGBA purple = new NormalizedRGBA();
    private final NormalizedRGBA green = new NormalizedRGBA();

    HardwareMap hardwaremap;

    public Intake(HardwareMap hardwareMap){
        purple.red = 0.68f;
        purple.green = 0.29f;
        purple.blue = 0.72f;
        purple.alpha = 0.95f;
        green.red = 0.17f;
        green.green = 0.73f;
        green.blue = 0.49f;
        green.alpha = 0.95f;

        this.hardwaremap = hardwaremap;
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
            t.addLine("Uptake result: Full Intake");
            return;
        }

        String color = scanColor();
        if (color.equals("Purple")) {
            pgratio[0]++;
        } else if (color.equals("Green")) {
            pgratio[1]++;
        }
        order[toSet] = color;
        t.addLine("Uptake Result: " + color + " artifact uptaked into position " + toSet);

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
                    tmp = order[2];
                    order[2] = order[1];
                    order[1] = order[0];
                    order[0] = tmp;
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
                t.addLine("Eject result: arget found at pos 1");
                tmp = order[1];
                order[1] = order[2];
                order[2] = order[0];
                order[0] = tmp;
                // ROTATOR FUNCTION
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
        order[0] = order[1];
        order[1] = order[2];
        order[2] = null;
        // Depending on how the intake storage is built, may need to ROTATE 1 unit
        t.addLine("P:G - " + pgratio[0] + ":" + pgratio[1]);
    }

    public TelemetryPacket sendTelemetry(){
        return t;
    }
}
