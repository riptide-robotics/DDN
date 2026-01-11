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

// Imports to sync

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Intake{
    /**
     * This is the combination of the intake and spindex subassembilies.
     * ...
     */
    TelemetryPacket t = new TelemetryPacket();

    NormalizedColorSensor colorSensor;
    DcMotor intakeMotor;
    ServoImplEx spindexServo;
    Servo spindexArm;

    public static char SLOT_0 = 'b';
    public static char SLOT_1 = 'b';
    public static char SLOT_2 = 'b';

    public static double ballsShot = 0;

    public static int diff;
    public static int currAngle;

    public static UnshiftedPositions currentState = UnshiftedPositions.SLOT_0_SHOOT;

    public boolean checkClose = false;

    public static float gain = 30;

    private final double spindexRange = 900;

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

        UnshiftedPositions(int pos) {
            this.posUnshifted = pos;
        }
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

    public Intake(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "REVcolorSensor");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        spindexServo = hardwareMap.get(ServoImplEx.class, "spindexServo");
        spindexServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        spindexArm = hardwareMap.get(Servo.class, "bootkicker");
        spindexServo.setDirection(Servo.Direction.FORWARD);
        spindexArm.setDirection(Servo.Direction.REVERSE);
    }

    public void initColorSensor() {
        colorSensor.setGain(gain);
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }
    }

    public void initSpindex() {
        currAngle = 450;
        setSpindexPosition(currAngle);
        ballsShot = 0;
        currentState = UnshiftedPositions.SLOT_0_RECEIVE;
        SLOT_0 = 'b';
        SLOT_1 = 'b';
        SLOT_2 = 'b';
    }

    public void increaseCount(){
        ballsShot++;
    }

    // just incase driver messes up
    public void decreseCount(){
        ballsShot--;
    }

    public double getCount(){
        return ballsShot;
    }

    public void resetCount() {
        ballsShot = 0;
    }
    /**
     * Gain is...
     */
    public void setGain(float gain) {
        colorSensor.setGain(gain);
    }

    // Sets the power of the intake motor, NOT THE SPINDEX
    public void spin(double p) {
        intakeMotor.setPower(p);
    }



    public char checkColor() {
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

    /**
     * NOTE NEEDS TO BE REFACTORED TO USE checkColor()
     *
     * @return
//     */
//    public slotStatus currColor() {
//        NormalizedRGBA colors = colorSensor.getNormalizedColors();
//        slotStatus status = slotStatus.BLANK;
//        if ((colors.red < PURPLE_R + PURPLE_R_STDEV
//                &&
//                colors.red > PURPLE_R - PURPLE_R_STDEV)
//                &&
//                (colors.green < PURPLE_G + PURPLE_G_STDEV
//                        &&
//                        colors.green > PURPLE_G - PURPLE_G_STDEV)
//                &&
//                (colors.blue < PURPLE_B + PURPLE_B_STDEV
//                        &&
//                        colors.blue > PURPLE_B - PURPLE_B_STDEV)) {
//            status = slotStatus.PURPLE;
//        } else if ((colors.red < GREEN_R + GREEN_R_STDEV
//                &&
//                colors.red > GREEN_R - GREEN_R_STDEV)
//                &&
//                (colors.green < GREEN_G + GREEN_G_STDEV
//                        &&
//                        colors.green > GREEN_G - GREEN_G_STDEV)
//                &&
//                (colors.blue < GREEN_B + GREEN_B_STDEV
//                        &&
//                        colors.blue > GREEN_B - GREEN_B_STDEV)) {
//            status = slotStatus.GREEN;
//        } else {
//            checkClose = true;
//        }
//
//        // Since I cant accurately tell what color the ball is when faced with a hole, I just check if there is a ball
//
//        return status;
//    }

    public char slot0CurrColor (){
        return SLOT_0;
    }

    public char slot1CurrColor (){
        return SLOT_1;
    }

    public char slot2CurrColor (){
        return SLOT_2;
    }

    public void goTo(UnshiftedPositions goal) {
        diff = goal.posUnshifted - currentState.posUnshifted;

        if (diff < -180) {
            diff += 360;
        }

        if (diff > 180) {
            diff -= 360;
        }

        int newAngle = currAngle + diff ;
        if (newAngle < 0) {
            newAngle += 360;
        }
        if (newAngle >= spindexRange) {
            newAngle -= 360;
        }
        currAngle = newAngle;
        setSpindexPosition(newAngle);
        currentState = goal;

    }


    public double getNextIntakeSlot() {
        if (SLOT_0 == 'b') return 0;
        if (SLOT_1 == 'b') return 1;
        if (SLOT_2 == 'b') return 2;
        else {
            return -1;
        }
    }

    public int getNextOuttakeSlot() {
        if (SLOT_0 != 'b') return 0;
        if (SLOT_1 != 'b') return 1;
        if (SLOT_2 != 'b') return 2;
        return -1;
    }

    @Deprecated
    public double fiveTurnToServo(double angle) {
        //900 instead of 1800 because we using 2:1 gear ratio (180*5)
        return angle <= 887 && angle >= 0 ? angle / 887 : angle > 887 ? 1 : 0;
    }

    /**
     * a utility function to convert an actual angle to a range between [0, 1]
     * For any type of servo, spindex
     *
     * @param angle an angle, will be clamped to [0, spindexRange]
     * @return a double in the range [0, 1]
     */
    public double angleToServo(double angle) {
        double max = spindexRange;
        double min = 0;
        return angle > min && angle < max ? angle / spindexRange : angle > max ? 1 : 0;
    }

    @Deprecated
    public void spindexPos2to1Gear(double pos) {
        spindexServo.setPosition(fiveTurnToServo(pos));
    }

    /**
     * Sets the spindex servo to an angle position.
     *
     * @param pos a degree measure between [0 and spindex Range]
     */
    public void setSpindexPosition(double pos){
        spindexServo.setPosition(angleToServo(pos));
    }

    /**
     * Sets the spindex Servo to the center of it's range (spindexRange / 2)
     */
    public void zeroSpindex(){
       spindexServo.setPosition(0.5);
       currAngle = (int)spindexRange / 2;
       currentState = UnshiftedPositions.SLOT_0_RECEIVE;
    }

    public double currOuttakeSlot(){
        if (spindexCurrentPosition() == UnshiftedPositions.SLOT_0_SHOOT.posUnshifted) {return 0;}
        if (spindexCurrentPosition() == UnshiftedPositions.SLOT_1_SHOOT.posUnshifted) {return 1;}
        if (spindexCurrentPosition() == UnshiftedPositions.SLOT_2_SHOOT.posUnshifted) {return 2;}
        else {return -1;}
    }

    public double spindexCurrentPosition() {
        return spindexServo.getPosition() * spindexRange;
    }

    public double bootKickCurrPos() {
        return spindexArm.getPosition();
    }

    public void bootkick(double pos) {
        spindexArm.setPosition(pos);

    }
}