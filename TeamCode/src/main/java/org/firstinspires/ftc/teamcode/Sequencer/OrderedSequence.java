package org.firstinspires.ftc.teamcode.Sequencer;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Stream;

public class OrderedSequence extends SequenceBase{
    LinkedList<SequenceBase> sequences = new LinkedList<>();
    Stream<SequenceBase> sequenceBaseStream = sequences.stream();
    int stage = 0;
    public OrderedSequence() {
        super(() -> {}); //has to be edited after constructed lol

        this.runnable = () -> {
            Iterator<SequenceBase> iterate = sequenceBaseStream.iterator();
            while (iterate.hasNext()) {
                SequenceBase base =  iterate.next();
                if (!(base.canExecute = base.iterationLoop())) break;
                base.runnable.run();

            }
        };
    }

    @Override
    public boolean iterationLoop() {
        return false;
    }
}
