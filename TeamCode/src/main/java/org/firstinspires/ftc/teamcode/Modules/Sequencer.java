package org.firstinspires.ftc.teamcode.Modules;

import java.util.ArrayList;

//practically copy-pasting, don't blame me -

public class Sequencer {

    public Sequencer(){};

    public interface Action{
        void action();
    }

    public static class TimedAction{
        Action a;
        final double startTime;
        double elapsedTime;


        public TimedAction(Action a, double delay) {
            this.a = a;
            this.elapsedTime = delay;
            this.startTime = System.currentTimeMillis();
        }
    }

    ArrayList<TimedAction> actions;

    public void addAction(TimedAction a){
        actions.add(a);
    }

    public void addAction(Action a, double delayINSECONDS){
        addAction(new TimedAction(a,delay/1000));
    }
    /**Haven't figured out how to run loop automatically*/
    public void loop() {
        for (int i = 0; i < actions.size(); i++) {
            if (System.currentTimeMillis() - actions.get(i).startTime > actions.get(i).elapsedTime) {
                actions.get(i).a.action();
                actions.remove(i);
                i--; //step i back since we just removed what we were working on.
                // Interesting but useless side effect is we know how many actions we have in waiting
            }
        }
    }
}
