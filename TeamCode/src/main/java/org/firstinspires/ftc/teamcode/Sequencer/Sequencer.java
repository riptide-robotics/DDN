package org.firstinspires.ftc.teamcode.Sequencer;

import org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes.SequenceBase;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Sequencer {
    /*
    saving and loading sequences




     */


    public List<SequenceBase> sequence;


    public Sequencer() {
        sequence = new ArrayList<>();
    };

    /**Add any sequence to the list, processing as necessary/*/
    public void addSequence(SequenceBase base) {
        sequence.add(base);
    }

    public Map<SequenceBase, List<SequenceBase>> waiting = new HashMap<SequenceBase, List<SequenceBase>>();

    /**
     * @param watched The sequence that the other is waiting on.
     * @param add The sequence to add after watched.
     * */
    public void addAfterExecution(SequenceBase watched, SequenceBase add) {
        if (!waiting.containsKey(watched)) waiting.put(watched, new ArrayList<>());
        waiting.get(watched).add(add);
    }

    @SuppressWarnings("AssignmentUsedAsCondition") //PLEASE DONT CHANGE IT
    /** Executes the Sequencer's stored sequences, handling all conditions. Place this in your loop. Redundant in SequencedOpMode.*/
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

            boolean modified = true;

            while (modified) {
                modified = false;
                for (Map.Entry<SequenceBase, List<SequenceBase>> pair : waiting.entrySet()) {
                    if (pair.getKey().canExecute) {
                        modified = true;
                        sequence.addAll(pair.getValue());
                        pair.setValue(new ArrayList<>());
                    }
                }
                waiting.entrySet().removeIf(entry -> entry.getValue().isEmpty());

            }
        }
    }
}
