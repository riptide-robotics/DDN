package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;


import org.firstinspires.ftc.teamcode.Sequencer.Values;

/** The base class for every Sequence. Abstract; may not be instantiated directly. However, refer to this to access any given sequence.*/
public abstract class SequenceBase {
    protected SequenceDataLambda runnable;

    public SequenceDataAccessor accessor;
    public boolean end = false;
    public boolean canExecute = false;
    public Values values = new Values();


    public SequenceBase(SequenceDataLambda runnable) {
        this.runnable = runnable;
        this.accessor = new SequenceDataAccessor(this);
    }

    public void run() {
        accessor.retrieve();
        runnable.run(accessor);
        accessor.assign();
    }
    /**@return Whether to execute the task. */
    public abstract boolean iterationLoop();

    public interface SequenceDataLambda {
        void run(SequenceDataAccessor accessor);
    }
}
