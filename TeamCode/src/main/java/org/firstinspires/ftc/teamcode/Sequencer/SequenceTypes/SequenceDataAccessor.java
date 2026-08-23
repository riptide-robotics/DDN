package org.firstinspires.ftc.teamcode.Sequencer.SequenceTypes;

import org.firstinspires.ftc.teamcode.Sequencer.Values;

public class SequenceDataAccessor {
    public boolean end = false;
    public boolean canExecute = false;
    public Values values = new Values();

    public SequenceBase base;

    public SequenceDataAccessor(SequenceBase base) {
        this.base = base;
    }

    protected void assign() {
        base.end = end;
        base.canExecute = canExecute;
        base.values = values;
    }

    protected void retrieve() {
        end = base.end;
        canExecute = base.canExecute;
        values = base.values;
    }
}
