package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;

/**
 * This sequence simply will run after another sequence. <br>
 * This may be on this cycle or the next.
 * */
public class LinkedSequence extends SequenceBase {

    public SequenceBase link;

    public LinkedSequence(SequenceDataLambda runnable, SequenceBase link) {
        super(runnable);
        this.link = link;
    }

    @Override
    public boolean iterationLoop() {
        return link.canExecute;
    }
}
