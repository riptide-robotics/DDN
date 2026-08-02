package org.firstinspires.ftc.teamcode.Sequencer;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.RepeatedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.SequenceBase;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.TimedSequenceBase;

/**This enum is a way of storing sequences that are often run.*/
public enum CommonSequences {
    TEMPLATE(new RepeatedSequence(() -> {
        System.out.println("Template sequence");
    },50,100));

    public SequenceBase sequenceBase;
    private CommonSequences(SequenceBase sequence) {
        this.sequenceBase = sequence;
    }
    /**Use before every call to a sequence that has already been used.*/
    public void refreshSequence() {
        sequenceBase.end = false;
        sequenceBase.canExecute = false;
        if (sequenceBase instanceof TimedSequenceBase) {
            TimedSequenceBase timedSequenceBase = (TimedSequenceBase) sequenceBase;
            timedSequenceBase.destinationTimer = timedSequenceBase.msDelay + System.currentTimeMillis();
        }
    }



}
