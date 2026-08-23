package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.stream.Stream;

/**The OrderedSequence is a system that will attempt to run sequences in a specific order. <br>
 * If it is forbidden to execute by the next sequence's logic, it will stop executing for this cycle. <br>
 * It will pick up from the same spot next cycle, potentially doing nothing if it is forbidden again. <br>
 * If it reaches its end, it will not loop back. It must be reinstantiated or recreated with OrderedSeqConstructor.
 * */
public class OrderedSequence extends SequenceBase {
    LinkedList<SequenceBase> sequences = new LinkedList<>();
    Stream<SequenceBase> sequenceBaseStream = sequences.stream();
    Iterator<SequenceBase> iterate;

    SequenceBase currBase = null;
    public OrderedSequence() {
        super((a) -> {}); //has to be edited after constructed lol
        iterate = sequenceBaseStream.iterator();
        this.runnable = (a) -> {
            if (currBase == null) currBase = iterate.next();

            while (iterate.hasNext()) {
                if (!(currBase.canExecute = currBase.iterationLoop())) break;

                currBase.run();

                if (!currBase.end) break;
                else currBase = iterate.next();
            }
            this.end = !iterate.hasNext();
        };
    }

    public OrderedSequence(SequenceBase... bases) {
        this();
        sequences.addAll(Arrays.asList(bases));
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

    public interface SeqLambda {
        OrderedSequence run();
    }


    /**
     * OrderedSeqConstructor stores a single OrderedSequence constructor within a lambda. <br>
     * It uses this to create a new instance as necessary, bypassing the usual limit of one cycle.
     **/
    public class OrderedSeqConstructor {
        public SeqLambda seq;

        public OrderedSeqConstructor(SeqLambda seq) {
            this.seq = seq;
        }

        public OrderedSequence assign(SeqLambda s) {
            this.seq = s;
            return construct();
        }

        public OrderedSequence construct() {
            return seq.run();
        }
    }
}
