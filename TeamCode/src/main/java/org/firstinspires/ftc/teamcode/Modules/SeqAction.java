package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Modules.Sequencer;

public class SeqAction {
    public Sequencer.Action a;
    public String name;
    public final double startTime;
    public double elapsedTime;
    SeqAction(Sequencer.Action a, double elapsedTime, String name) {
        startTime = (double) System.currentTimeMillis() / 1000;
        this.elapsedTime = elapsedTime;
        this.name = name;
        this.a = a;
    }
}