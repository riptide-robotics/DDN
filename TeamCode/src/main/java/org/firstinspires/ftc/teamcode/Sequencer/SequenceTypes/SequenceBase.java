package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;


import org.firstinspires.ftc.teamcode.Sequencer.Values;

/** The base class for every Sequence. Abstract; may not be instantiated directly. However, refer to this to access any given sequence.*/
public abstract class SequenceBase {
    public Runnable runnable;
    public boolean end = false;
    public boolean canExecute;
    public Values values = new Values();
    public SequenceBase(Runnable runnable) {
        this.runnable = runnable;
    }

    /**@return Whether to execute the task. */
    public abstract boolean iterationLoop();

    public void setTerminationStatus(boolean terminate) {
        end = terminate;
    }
}
