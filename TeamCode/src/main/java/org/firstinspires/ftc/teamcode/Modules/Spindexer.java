package org.firstinspires.ftc.teamcode.Modules;


import androidx.annotation.NonNull;

public class Spindexer {
    static int current = 0;
    public enum artifactColor {
        EMPTY, GREEN, PURPLE
    }
    static artifactColor slot0 = artifactColor.EMPTY;
    static artifactColor slot1 = artifactColor.EMPTY;
    static artifactColor slot2 = artifactColor.EMPTY;

    public Spindexer() {
        throw new UnsupportedOperationException("Why are you running this");
    }
    public void setArtifact(int set, artifactColor color) {
        if (set == 0) slot0 = color;
        if (set == 1) slot1 = color;
        if (set == 2) slot2 = color;
    }
    @NonNull
    public String toString() {
        return "0: " + slot0.toString() + ", 1: " + slot1.toString() + ", 2: " + slot2.toString();
    }
    public void rotateToPostiton() {
        throw new UnsupportedOperationException("Why are you running this");
    }
}