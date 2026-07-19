package org.firstinspires.ftc.teamcode.Sequencer;

public class RepeatedSequence extends TimedSequenceBase {
    long msRepeatDelay;
    boolean prevExecuted = false;
    public RepeatedSequence(Runnable runnable, long msDelay, long msRepeatDelay) {
        super(runnable, msDelay);
        this.msRepeatDelay = msRepeatDelay;
    }

    @Override
    public boolean iterationLoop() {
        boolean value =  System.currentTimeMillis() < destinationTimer;
        if (value) destinationTimer += msRepeatDelay;
        return value;
    }

}
