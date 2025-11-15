package org.firstinspires.ftc.teamcode.Modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

public class Spindexer {
    static Slot currentSlot = Slot.SLOT0;

    public enum artifactColor {
        EMPTY, GREEN, PURPLE
    }
    public enum Slot {
        SLOT0(0), SLOT1(120), SLOT2(240);
        int rotation;
        Slot(int i) {
            rotation = i;
        }
        /**In degrees.*/
        public static Slot getSlot(int rotation) {
            rotation = rotation % 360;
            if (rotation == 0) return Slot.SLOT0;
            if (rotation == 120) return Slot.SLOT1;
            if (rotation == 240) return Slot.SLOT2;
            return null;
        }
    }
    static artifactColor slot0 = artifactColor.EMPTY;
    static artifactColor slot1 = artifactColor.EMPTY;
    static artifactColor slot2 = artifactColor.EMPTY;

    static int currentLocation = 0;
    static Servo servo;

    public Spindexer(HardwareMap hardwareMap) {
        servo = hardwareMap.servo.get("spindex");


        throw new UnsupportedOperationException("Why are you running this");
    }
    public void setArtifact(Slot slot, artifactColor color) {
        if (slot == Slot.SLOT0) slot0 = color;
        if (slot == Slot.SLOT1) slot1 = color;
        if (slot == Slot.SLOT2) slot2 = color;
    }


    public void rotateToPosition(Slot slot) {
        //figure this out later

        throw new UnsupportedOperationException("How did you get this");
    }
    /**Can return an empty array if the color is not present.*/
    public List<Slot> getColorPositions(artifactColor color) {
        boolean inSlot0 = slot0 == color;
        boolean inSlot1 = slot1 == color;
        boolean inSlot2 = slot2 == color;

        List<Slot> set = new ArrayList<>();

        if (inSlot0) set.add(Slot.SLOT0);
        if (inSlot1) set.add(Slot.SLOT1);
        if (inSlot2) set.add(Slot.SLOT2);

        return set;
    }

    public artifactColor getColorAtPosition(Slot slot) {
        if (slot == Slot.SLOT0) return slot0;
        if (slot == Slot.SLOT1) return slot1;

        return slot2;
    }

    @NonNull
    public String toString() {
        return "0: " + slot0.toString() + ", 1: " + slot1.toString() + ", 2: " + slot2.toString();
    }
}