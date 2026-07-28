package org.firstinspires.ftc.teamcode.Sequencer;

public class OrderedSequence extends SequenceBase{
    public OrderedSequence(Runnable runnable) {
        super(runnable);
    }

    @Override
    public boolean iterationLoop() {
        return false;
    }
}
