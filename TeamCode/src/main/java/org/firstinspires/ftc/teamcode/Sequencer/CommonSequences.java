package org.firstinspires.ftc.teamcode.Sequencer;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.DataWatcherSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.LinkedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.RepeatedSequence;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.SequenceBase;
import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.TimedSequenceBase;

/**This enum is a way of storing sequences that are often run.*/
public enum CommonSequences {
    TEMPLATE(new RepeatedSequence(() -> {
        System.out.println("Template sequence");
    },50,100));

    public SequenceBase sequence;
    private CommonSequences(SequenceBase sequence) {
        this.sequence = sequence;
    }
    /**Use before every call to a sequence that has already been used.*/
    public void refreshSequence() {
        sequence.end = false;
        sequence.canExecute = false;
        if (sequence instanceof TimedSequenceBase) {
            TimedSequenceBase timedSequenceBase = (TimedSequenceBase) sequence;
            timedSequenceBase.destinationTimer = timedSequenceBase.msDelay + System.currentTimeMillis();
        }
    }
    public static SequenceBase refresh(CommonSequences seq) {
        seq.refreshSequence();
        return seq.sequence;
    }



}
