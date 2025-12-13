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
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_ONE_PIKCUP_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_ONE_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_THREE_PIKCUP_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_THREE_SHOOT_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_TWO_PIKCUP_POS;
import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_TWO_SHOOT_POS;

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

public class Intake{
    TelemetryPacket t = new TelemetryPacket();

    NormalizedColorSensor colorSensor;
    DcMotor intakeMotor;
    ServoImplEx spindexServo;
    Servo spindexArm;

    public static slotStatus SLOT_0 = slotStatus.BLANK;
    public static slotStatus SLOT_1 = slotStatus.BLANK;
    public static slotStatus SLOT_2 = slotStatus.BLANK;

    public static int diff;
    public static int currAngle;

    public static UnshiftedPositions currentState = UnshiftedPositions.SLOT_0_SHOOT;

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
        spindexArm.setDirection(Servo.Direction.REVERSE);
    }

    public void setGain (float gain){
        colorSensor.setGain(gain);
    }

    public char scanColor() {
        NormalizedRGBA detectedColor = colorSensor.getNormalizedColors();

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

    public void spin(double p) {
        intakeMotor.setPower(p);
    }

    public TelemetryPacket sendTelemetry(){
        return t;
    }

    public double fiveTurnToServo(double angle){
        //900 instead of 1800 because we using 2:1 gear ratio (180*5)
        return angle <= 887 && angle >= 0 ? angle / 887 : angle > 887 ? 1 : 0;
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
        }

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
        }

        // Since I cant accurately tell what color the ball is when faced with a hole, I just check if there is a ball

        return status;
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


    public void bootkick(double pos){
        spindexArm.setPosition(pos);

    }
}