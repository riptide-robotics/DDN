package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;

import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.stream.Stream;

/**The OrderedSequence is a system that will attempt to run sequences in a specific order. <br>
 * If it is forbidden to execute by the next sequence's logic, it will stop executing for this cycle. <br>
 * It will pick up from the same spot next cycle, potentially doing nothing if it is forbidden again.
 * */
public class OrderedSequence extends SequenceBase {
    LinkedList<SequenceBase> sequences = new LinkedList<>();
    Stream<SequenceBase> sequenceBaseStream = sequences.stream();
    Iterator<SequenceBase> iterate;
    public OrderedSequence() {
        super(() -> {}); //has to be edited after constructed lol
        iterate = sequenceBaseStream.iterator();
        this.runnable = () -> {
            while (iterate.hasNext()) {
                SequenceBase base =  iterate.next();
                if (!(base.canExecute = base.iterationLoop())) break;
                base.runnable.run();
            }
            if (!iterate.hasNext()) {
                resetPosition();
            }
        };
    }
    /**Chainable.*/
    public OrderedSequence add(int slot, SequenceBase base) {
        sequences.add(slot, base);
        return this;
    }
    /**Chainable.*/
    public OrderedSequence add(SequenceBase base) {
        sequences.add(base);
        return this;
    }
    /**Chainable.*/
    public OrderedSequence remove(int slot) {
        sequences.remove(slot);
        return this;
    }
    /**Chainable.*/
    public OrderedSequence addAll(SequenceBase... base) {
        sequences.addAll(Arrays.asList(base));
        return this;
    }


    public void resetPosition() {
        sequenceBaseStream = sequences.stream();
        iterate = sequenceBaseStream.iterator();
    }

    @Override
    public boolean iterationLoop() {
        return true;
    }
}
