package org.firstinspires.ftc.teamcode.Modules;


public class Spindex {
    int current = 0;
    enum artifactColor {
        EMPTY, GREEN, PURPLE
    }
    artifactColor slot0 = artifactColor.EMPTY;
    artifactColor slot1 = artifactColor.EMPTY;
    artifactColor slot2 = artifactColor.EMPTY;

    public Spindex() {
        throw new UnsupportedOperationException("Why are you running this");
    }
    public void setArtifact(int set, artifactColor color) {
        if (set == 0) slot0 = color;
        if (set == 1) slot1 = color;
        if (set == 2) slot2 = color;
    }
}