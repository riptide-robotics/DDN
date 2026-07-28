package org.firstinspires.ftc.teamcode.Sequencer;

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
}
