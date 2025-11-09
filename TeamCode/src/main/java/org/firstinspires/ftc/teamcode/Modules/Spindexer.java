package org.firstinspires.ftc.teamcode.Modules;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spindexer {
    static Slot currentSlot = Slot.slot0;
    public enum artifactColor {
        EMPTY, GREEN, PURPLE
    }
    public enum Slot {
        slot0, slot1, slot2
    }
    static artifactColor slot0 = artifactColor.EMPTY;
    static artifactColor slot1 = artifactColor.EMPTY;
    static artifactColor slot2 = artifactColor.EMPTY;

    static int currentLocation = 0;

    static final int loc0 = 0;
    static final int loc1 = 120;

    static final int loc2 = 240;

    static DcMotor motor;
    public Spindexer(HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get("spindexer");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        throw new UnsupportedOperationException("Why are you running this");
    }
    public void setArtifact(Slot slot, artifactColor color) {
        if (slot == Slot.slot0) slot0 = color;
        if (slot == Slot.slot1) slot1 = color;
        if (slot == Slot.slot2) slot2 = color;
    }
    @NonNull
    public String toString() {
        return "0: " + slot0.toString() + ", 1: " + slot1.toString() + ", 2: " + slot2.toString();
    }
    public void rotateToPosition(Slot slot) {




        throw new UnsupportedOperationException("How did you get this");
    }
}