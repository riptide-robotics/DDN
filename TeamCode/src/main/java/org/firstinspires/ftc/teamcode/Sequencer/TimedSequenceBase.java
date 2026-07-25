package org.firstinspires.ftc.teamcode.Sequencer;

public class TimedSequenceBase extends SequenceBase {
    public long destinationTimer;
    public TimedSequenceBase(Runnable runnable, long msDelay) {
        super(runnable);
        // super(runnable);
        destinationTimer = System.currentTimeMillis() + msDelay;
    }

    @Override
    public boolean iterationLoop() {
        return end = System.currentTimeMillis() > destinationTimer;
    }
}
