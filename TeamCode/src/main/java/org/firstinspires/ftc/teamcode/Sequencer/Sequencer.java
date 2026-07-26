package org.firstinspires.ftc.teamcode.Sequencer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Sequencer {
    /*
    saving and loading sequences




     */


    public List<SequenceBase> sequence;


    public Sequencer() {
        sequence = new ArrayList<>();
    };
    public void addTimedSequenceBase(Runnable runnable, long msDelay) {
        sequence.add(new TimedSequenceBase(runnable, msDelay));
    }

    public void addRepeatedSequence(Runnable runnable, long msDelay,long msRepeatDelay) {
        sequence.add(new RepeatedSequence(runnable, msDelay, msRepeatDelay));
    }

    @SuppressWarnings("AssignmentUsedAsCondition") //PLEASE DONT CHANGE IT
    /** Executes the Sequencer's stored sequences, handling all conditions. Place this in your loop.*/
    public synchronized void iterate() {

        {
            int[] remove = new int[sequence.size()];
            int  deletedCount = 0;

            for (int i = 0; i < sequence.size(); i++) {
                SequenceBase seqbase = sequence.get(i);

                if (seqbase.canExecute = seqbase.iterationLoop()) seqbase.runnable.run();
                if (seqbase.end) remove[deletedCount++] = i;
            }
            remove = Arrays.copyOf(remove, deletedCount + 1);

            while(remove.length > 0) {
                sequence.remove(remove[remove.length - 1]);
                remove = Arrays.copyOf(remove, remove.length - 1);
            }
        }
    }
}
