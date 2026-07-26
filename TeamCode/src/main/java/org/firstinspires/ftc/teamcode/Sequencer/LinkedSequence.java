package org.firstinspires.ftc.teamcode.Sequencer;

import kotlin.sequences.Sequence;

public class LinkedSequence extends SequenceBase {

    public SequenceBase link;

    public LinkedSequence(Runnable runnable, SequenceBase link) {
        super(runnable);
        this.link = link;
    }

    @Override
    public boolean iterationLoop() {
        return link.canExecute;
    }
}
