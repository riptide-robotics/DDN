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
import static org.firstinspires.ftc.teamcode.riptideUtil.ROTATE_SPINDEX_ONCE;

// Imports to sync

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake{
    String[] order = new String[3];
    TelemetryPacket t = new TelemetryPacket();
    int[] pgratio = new int[2];

    NormalizedColorSensor colorSensor;
    DcMotor intakeMotor;
    Servo spindexServo;
    Servo spindexArm;
    float gain = 2; // ...... ANYWAY
    public enum positions {
        POS_1_SHOOT,
        POS_1_RECEIVE,
        POS_2_SHOOT,
        POS_2_RECEIVE,
        POS_3_SHOOT,
        POS_3_RECEIVE;

    }

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
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        spindexServo = hardwareMap.get(Servo.class, "spindexServo");
        spindexArm = hardwareMap.get(Servo.class, "spindexArm");
        spindexServo.setDirection(Servo.Direction.FORWARD);
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
    }

    public boolean CanBootKick(boolean canBootKick){
        return canBootKick;
    }

    public void movetoEmptySlot() {
        int emptySlot = -1;
        for (int i = 0; i < order.length; i++) {
            if (order[i] == null) {
                emptySlot = i;
                break;
            }
        }

        if (emptySlot == -1) {
            t.addLine("Intake failed: Full intake");
            return;
        }

        while (emptySlot != 0) {


//            ElapsedTime timer = new ElapsedTime();
//            timer.reset();
//            while (timer.milliseconds() < slotMoveTimeMs) {
//                spinSpindex(spindexPower);
//            }

            rotateOne(true);
            emptySlot--;
            CanBootKick(false);
        }
        rotateOne(false);
        CanBootKick(true);
        t.addLine("Empty slot aligned at intake");

        String color = scanColor();
        if (color.equals("Purple")) {
            pgratio[0]++;
        } else if (color.equals("Green")) {
            pgratio[1]++;
        }

        order[0] = color;
    }

    public void BootKick(double pos){
        spindexArm.setPosition(pos);
    }

    public void ResetBootKick(double pos){
        spindexArm.setPosition(pos);
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
            rotateSpindexOnce();
            order[0] = order[1];
            order[1] = order[2];
            order[2] = tmp;
        } else {
            order[0] = order[2];
            order[2] = order[1];
            order[1] = tmp;
        }

    }

    public void spin(double p) {
        intakeMotor.setPower(p);
    }
    public void rotateSpindexOnce() {
        double currPos = spindexServo.getPosition() * 900;
        if (900 > currPos + ROTATE_SPINDEX_ONCE){
            spindexServo.setPosition(fiveTurnToServo(currPos - ROTATE_SPINDEX_ONCE));
            t.addLine("Rotating Spindex to " + fiveTurnToServo(currPos - ROTATE_SPINDEX_ONCE) + "(deg)");
        }
        if (-900 < currPos - ROTATE_SPINDEX_ONCE){
            spindexServo.setPosition(fiveTurnToServo(currPos + ROTATE_SPINDEX_ONCE));
            t.addLine("Rotating Spindex " + fiveTurnToServo(currPos + ROTATE_SPINDEX_ONCE) + "(deg)");
        }

    }

    // THIS IS PURELY FOR TUNING DELETE AFTER
    public void spindexPos(double pos){
        spindexServo.setPosition(fiveTurnToServo(pos));
    }

//    public void transferToggle() {
//        if (spindexServo.getPower() > 0) {
//            spinSpindex(0);
//        } else {
//            spinSpindex(1);
//        }
//    }

    private boolean isTransferOpen = false;
    private boolean isTransferClosed = false;
    public void openTransfer(double pos){
        spindexArm.setPosition(/* SPINDEX_ARM_UP */ pos);
        isTransferOpen = true;
        isTransferClosed = false;
    }
    public void closeTransfer(double pos){
        spindexArm.setPosition(/* SPINDEX_ARM_RESTING */ pos);
        isTransferOpen = false;
        isTransferClosed = true;
    }

    public TelemetryPacket sendTelemetry(){
        return t;
    }

    public double fiveTurnToServo(double angle){
        //900 instead of 1800 because we using 2:1 gear ratio (180*5)
        return angle <= 900 && angle >= 0 ? angle / 900 : angle > 900 ? 1 : 0;
    }
    public void setMotifOrder(char[] order) {
        if (order.length != 3) throw new RuntimeException("you put a bad order into an unsupported system. Congratulations.");
        throw new UnsupportedOperationException("This does not work!");
    }
    public void rotateToPositionShoot(int pos) {
        throw new UnsupportedOperationException("This does not work!");
    }
    public void rotateToPositionRecieve(int pos) {
        throw new UnsupportedOperationException("This does not work!");
    }
    public void setIntakerPower(double power) {
        throw new UnsupportedOperationException("This does not work!");
    }
    public void Shoot(int position){
        // dunno why we decided that this would not be camel case, guess its that important
        // ...
        // ANYWAYS this does not work
        throw new UnsupportedOperationException("This does not work!");
    }
    public positions getCurrentState(){
        //just like everything else
        throw new UnsupportedOperationException("This does not work!");
    }
    /**Does not actually get anything, just moves spindexer. Eventually. Right now, its only purpose is to throw an exception.*/
    public void hasReceivedArtifact(positions pos){
        //use colorsensor,
        //if colorsensor sees color
        //Sets the receiving position to indicate that an artifact of the colorsensor's output is in that position
        throw new UnsupportedOperationException("This does not work!");
    }
    public boolean hasRecievedArtifact(String color) {
        return false;
    }
}