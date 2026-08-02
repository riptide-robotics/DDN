package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;

/**
 * This class is a basic sequence that waits for a segment of time, then executes.
 * */
public class TimedSequenceBase extends SequenceBase {
    public long destinationTimer;
    public long msDelay;

    public TimedSequenceBase(Runnable runnable, long msDelay) {
        super(runnable);
        // super(runnable);
        destinationTimer = System.currentTimeMillis() + msDelay;
        this.msDelay = msDelay;
    }

    @Override
    public boolean iterationLoop() {
        return end = System.currentTimeMillis() > destinationTimer;
    }
}
