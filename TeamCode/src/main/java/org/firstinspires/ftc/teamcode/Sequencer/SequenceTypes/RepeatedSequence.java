package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;


/**
 * This sequence starts off just like the TimedSequenceBase it extends. <br>
 * However, after execution, it waits by a new delay.
 * */
public class RepeatedSequence extends TimedSequenceBase {
    long msRepeatDelay;
    boolean prevExecuted = false;
    /**
     * @param msDelay the starting delay.
     * @param msRepeatDelay the delay between one execution from the next.
     * */
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
