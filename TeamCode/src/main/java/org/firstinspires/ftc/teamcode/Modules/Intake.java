package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_B;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_B_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_B_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_B_STDEV_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_G;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_G_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_G_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_G_STDEV_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_R;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_R_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_R_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.GREEN_R_STDEV_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_B;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_B_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_B_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_B_STDEV_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_G;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_G_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_G_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_G_STDEV_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_R;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_R_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_R_STDEV;
import static org.firstinspires.ftc.teamcode.riptideUtil.PURPLE_R_STDEV_HOLE;
import static org.firstinspires.ftc.teamcode.riptideUtil.ROTATE_SPINDEX_ONCE;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_ONE_PIKCUP_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_ONE_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_THREE_PIKCUP_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_THREE_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_TWO_PIKCUP_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_TWO_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_RESTING;

// Imports to sync

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake{
    char[] order = new char[3];
    TelemetryPacket t = new TelemetryPacket();
    int[] pgratio = new int[2];

    NormalizedColorSensor colorSensor;
    DcMotor intakeMotor;
    ServoImplEx spindexServo;
    Servo spindexArm;
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
        spindexServo = hardwareMap.get(ServoImplEx.class, "spindexServo");
        spindexServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        spindexArm = hardwareMap.get(Servo.class, "bootkicker");
        spindexServo.setDirection(Servo.Direction.REVERSE);
    }

    public void setGain (float gain){
        colorSensor.setGain(gain);
    }

    public char scanColor() {
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
            return 'p';
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
            return 'g';
        } else {
            return 'b';
        }
    }

    public void uptakeTarget() {
        int toSet;
        if (order[0] == 'b') { // EMPTY INTAKE
            toSet = 0;
        } else if (order[1] == 'b') {
            toSet = 1;
        } else if (order[2] == 'b') {
            toSet = 2;
        } else { // FULL INTAKE
            toSet = -1;
        }
        if (toSet == -1) {
            t.addLine("Uptake result: Failure - Full Intake");
            return;
        }

        char color = scanColor();
        if (color == 'p') {
            pgratio[0]++;
        } else if (color == 'g') {
            pgratio[1]++;
        } else if (color == 'b') {
            t.addLine("Congrats: you found no color to uptake idiot.");
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
            if (order[i] == 'b') {
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

        char color = scanColor();
        if (color == 'g') {
            pgratio[0]++;
        } else if (color == 'p') {
            pgratio[1]++;
        } else if (color == 'b') {
            t.addLine("Congrats: There's no ball there.");
        }

        order[0] = color;
    }

    boolean moveSlotOneToPickup = false;
    boolean moveSlotTwoToPickup = false;
    boolean moveSlotThreeToPickup = true;


    public void slotOnePickup(){
        spindexPos2to1Gear(SLOT_ONE_PIKCUP_POS);
    }

    public void slotTwoPickup(){
        spindexPos2to1Gear(SLOT_TWO_PIKCUP_POS);
    }

    public void slotThreePickup(){
        spindexPos2to1Gear(SLOT_THREE_PIKCUP_POS);
    }

    public void slotOneOuttake(){
        spindexPos2to1Gear(SLOT_ONE_SHOOT_POS);
    }
    public void slotTwoOuttake(){
        spindexPos2to1Gear(SLOT_TWO_SHOOT_POS);
    }
    public void slotThreeOuttake(){
        spindexPos2to1Gear(SLOT_THREE_SHOOT_POS);
    }

    public void ejectArtifact(char colorReq){ // colorReq should b 'p' (purple) or 'g' (green)
        char tmp;
        if (order[0] == 'b' || order[0] != colorReq) {
            if (order[1] == 'b' || order[1] != colorReq) {
                if (order[2] == 'b' || order[2] != colorReq) {
                    t.addLine("Eject result: Failed search");
                    t.addLine("Action: Ejecting artifact");
                    tmp = order[0];
                    order[0] = 'b';
                    if (tmp == 'b') {
                        pgratio[0]++;
                        pgratio[0]--;
                    } else if (tmp == 'p') {
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
                    order[0] = 'b';
                    if (tmp == 'p') {
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
                order[0] = 'b';
                if (tmp == 'p') {
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
            order[0] = 'b';
            if (tmp == 'p') {
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
        char tmp = order[0];
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
        return angle <= 887 && angle >= 0 ? angle / 887 : angle > 887 ? 1 : 0;
    }
    public void setMotifOrder(char[] order) {
        if (order.length != 3) throw new RuntimeException("you put a bad order into an unsupported system. Congratulations.");
        this.order[0] = order[0]; this.order[1] = order[1]; this.order[2] = order[2]; //return;
        //throw new UnsupportedOperationException("This does not work!");
    } // Honestly just run the camera's scanMotifOrder in here and be done with it
      // Otherwise remember to run scanMotifOrder in robot or whatever to set
      // order to the correct obelisk value.
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



    // ******************************************
    //                SPINDEX
    // ******************************************

    public static slotStatus SLOT_0 = slotStatus.BLANK;
    public static slotStatus SLOT_1 = slotStatus.BLANK;
    public static slotStatus SLOT_2 = slotStatus.BLANK;

    public static int diff;
    public static int currAngle;

    public static UnshiftedPositions currentState = UnshiftedPositions.SLOT_0_SHOOT;

    public static int currentSlot = -1;

    public boolean checkClose = false;

    public static float gain = 30;

    public enum slotStatus {
        BLANK, GREEN, PURPLE
    }

    public enum UnshiftedPositions {
        SLOT_0_SHOOT(180),
        SLOT_1_SHOOT(60),
        SLOT_2_SHOOT(-60),
        SLOT_0_RECEIVE(0),
        SLOT_1_RECEIVE(120),
        SLOT_2_RECEIVE(-120);

        public final int posUnshifted;
        UnshiftedPositions(int pos) {this.posUnshifted = pos;}
    }

    public void initColorSensor(){
        colorSensor.setGain(gain);
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }

    public char checkColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        char currColor = 'b';
        if ((colors.red < PURPLE_R + PURPLE_R_STDEV
                &&
                colors.red > PURPLE_R - PURPLE_R_STDEV)
                &&
                (colors.green < PURPLE_G + PURPLE_G_STDEV
                        &&
                        colors.green > PURPLE_G - PURPLE_G_STDEV)
                &&
                (colors.blue < PURPLE_B + PURPLE_B_STDEV
                        &&
                        colors.blue > PURPLE_B - PURPLE_B_STDEV)) {
            currColor = 'p';

        } else if ((colors.red < GREEN_R + GREEN_R_STDEV
                &&
                colors.red > GREEN_R - GREEN_R_STDEV)
                &&
                (colors.green < GREEN_G + GREEN_G_STDEV
                        &&
                        colors.green > GREEN_G - GREEN_G_STDEV)
                &&
                (colors.blue < GREEN_B + GREEN_B_STDEV
                        &&
                        colors.blue > GREEN_B - GREEN_B_STDEV) && colors.alpha > 0.8) {
            currColor = 'g';
        } else {
            checkClose = true;
//                    currColor = 'b';
        }

//        if (checkClose){
//            if ((colors.red < PURPLE_R_HOLE + PURPLE_R_STDEV_HOLE
//                    &&
//                    colors.red > PURPLE_R_HOLE - PURPLE_R_STDEV_HOLE)
//                    &&
//                    (colors.green < PURPLE_G_HOLE + PURPLE_G_STDEV_HOLE
//                            &&
//                            colors.green > PURPLE_G_HOLE - PURPLE_G_STDEV_HOLE)
//                    &&
//                    (colors.blue < PURPLE_B_HOLE + PURPLE_B_STDEV_HOLE
//                            &&
//                            colors.blue > PURPLE_B_HOLE - PURPLE_B_STDEV_HOLE)) {
//                currColor = 'p';
//            } else if ((colors.red < GREEN_R_HOLE + GREEN_R_STDEV_HOLE
//                    &&
//                    colors.red > GREEN_R_HOLE - GREEN_R_STDEV_HOLE)
//                    &&
//                    (colors.green < GREEN_G_HOLE + GREEN_G_STDEV_HOLE
//                            &&
//                            colors.green > GREEN_G_HOLE - GREEN_G_STDEV_HOLE)
//                    &&
//                    (colors.blue < GREEN_B_HOLE + GREEN_B_STDEV_HOLE
//                            &&
//                            colors.blue > GREEN_B_HOLE - GREEN_B_STDEV_HOLE)) {
//                currColor = 'g';
//            } else{
//                currColor = 'b';
//            }
//            checkClose = false;
        //}
        return currColor;
    }

    public void goTo(UnshiftedPositions goal) {
        diff = goal.posUnshifted - currentState.posUnshifted;
        if (diff < -180) {
            diff += 360;
        }
        if (diff > 180) {
            diff -= 360;
        }

        int newAngle = currAngle + diff;
        if (newAngle < 0) {newAngle += 360;}
        if (newAngle > 887) {newAngle -= 360;}
        currAngle = newAngle;
        spindexPos2to1Gear(newAngle);
        currentState = goal;
    }

    public void initSpindex(){
        currAngle = 450;
        spindexPos2to1Gear(currAngle);
        currentState = UnshiftedPositions.SLOT_0_RECEIVE;
        SLOT_0 = slotStatus.BLANK;
        SLOT_1 = slotStatus.BLANK;
        SLOT_2 = slotStatus.BLANK;
    }


    public double getNextIntakeSlot() {
        if (SLOT_0 == slotStatus.BLANK) return 0;
        if (SLOT_1 == slotStatus.BLANK) return 1;
        if (SLOT_2 == slotStatus.BLANK) return 2;
        else {
            return -1;
        }
    }

    public int getNextOuttakeSlot() {
        if (SLOT_0 != slotStatus.BLANK) return 0;
        if (SLOT_1 != slotStatus.BLANK) return 1;
        if (SLOT_2 != slotStatus.BLANK) return 2;
        return -1;
    }

    public slotStatus currColor(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        slotStatus status = slotStatus.BLANK;
        if ((colors.red < PURPLE_R + PURPLE_R_STDEV
                &&
                colors.red > PURPLE_R - PURPLE_R_STDEV)
                &&
                (colors.green < PURPLE_G + PURPLE_G_STDEV
                        &&
                        colors.green > PURPLE_G - PURPLE_G_STDEV)
                &&
                (colors.blue < PURPLE_B + PURPLE_B_STDEV
                        &&
                        colors.blue > PURPLE_B - PURPLE_B_STDEV)) {
            status = slotStatus.PURPLE;
        } else if ((colors.red < GREEN_R + GREEN_R_STDEV
                &&
                colors.red > GREEN_R - GREEN_R_STDEV)
                &&
                (colors.green < GREEN_G + GREEN_G_STDEV
                        &&
                        colors.green > GREEN_G - GREEN_G_STDEV)
                &&
                (colors.blue < GREEN_B + GREEN_B_STDEV
                        &&
                        colors.blue > GREEN_B - GREEN_B_STDEV)) {
            status = slotStatus.GREEN;
        } else {
            checkClose = true;
//                    currColor = 'b';
        }

//        // Since I cant accurately tell what color the ball is when faced with a hole, I just check if there is a ball
//        if (checkClose){
//            if ((colors.red < PURPLE_R_HOLE + PURPLE_R_STDEV_HOLE
//                    &&
//                    colors.red > PURPLE_R_HOLE - PURPLE_R_STDEV_HOLE)
//                    &&
//                    (colors.green < PURPLE_G_HOLE + PURPLE_G_STDEV_HOLE
//                            &&
//                            colors.green > PURPLE_G_HOLE - PURPLE_G_STDEV_HOLE)
//                    &&
//                    (colors.blue < PURPLE_B_HOLE + PURPLE_B_STDEV_HOLE
//                            &&
//                            colors.blue > PURPLE_B_HOLE - PURPLE_B_STDEV_HOLE)) {
//                status = slotStatus.PURPLE;
//            } else if ((colors.red < GREEN_R_HOLE + GREEN_R_STDEV_HOLE
//                    &&
//                    colors.red > GREEN_R_HOLE - GREEN_R_STDEV_HOLE)
//                    &&
//                    (colors.green < GREEN_G_HOLE + GREEN_G_STDEV_HOLE
//                            &&
//                            colors.green > GREEN_G_HOLE - GREEN_G_STDEV_HOLE)
//                    &&
//                    (colors.blue < GREEN_B_HOLE + GREEN_B_STDEV_HOLE
//                            &&
//                            colors.blue > GREEN_B_HOLE - GREEN_B_STDEV_HOLE)) {
//                status = slotStatus.GREEN;
//            } else{
//                status = slotStatus.BLANK;
//            }
//            checkClose = false;
//        }

        return status;
    }
//
//    public int currSlot(){
//        if (spindexCurrentPosition() == UnshiftedPositions.SLOT_0_RECEIVE.posUnshifted || spindexCurrentPosition() == UnshiftedPositions.SLOT_0_SHOOT.posUnshifted){
//            currentSlot = 0;
//        }
//        if (spindexCurrentPosition() == UnshiftedPositions.SLOT_1_RECEIVE.posUnshifted || spindexCurrentPosition() == UnshiftedPositions.SLOT_1_SHOOT.posUnshifted){
//            currentSlot = 1;
//        }
//
//        if (spindexCurrentPosition() == UnshiftedPositions.SLOT_2_RECEIVE.posUnshifted || spindexCurrentPosition() == UnshiftedPositions.SLOT_2_SHOOT.posUnshifted){
//            currentSlot = 2;
//        }
//        return currentSlot;
//    }


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

    public void spindexPos2to1Gear(double pos){
        spindexServo.setPosition(fiveTurnToServo(pos));
    }
    public void spindexPos(double pos){
        spindexServo.setPosition(pos);
    }


    public double spindexCurrentPosition(){
        return spindexServo.getPosition() * 887;
    }

    public double bootKickCurrPos(){
        return spindexArm.getPosition();
    }


    public void BootKick(double pos){
        spindexArm.setPosition(pos);

    }
}