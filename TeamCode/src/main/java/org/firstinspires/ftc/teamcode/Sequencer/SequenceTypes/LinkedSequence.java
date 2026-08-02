package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;

/**This sequence simply will run after another sequence. This may be on this cycle or the next.*/
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
