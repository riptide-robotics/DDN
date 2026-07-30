package org.firstinspires.ftc.teamcode.Sequencer;

public class FullLoopSequence extends  RepeatedSequence{
    public FullLoopSequence(Runnable runnable) {
        super(runnable, 0, 0); //can only run once per iter anyways
    }
}
