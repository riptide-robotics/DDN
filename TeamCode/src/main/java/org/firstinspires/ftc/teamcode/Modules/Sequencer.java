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
            this.startTime = (double) System.currentTimeMillis() /1000;
        }
    }

    //A:one action
    //B:concurrent events set outtakePID speed to something
    //C:testing multiple sequences in order
    //D:test moving multiple things in order

    ArrayList<TimedAction> actions;

    public void addAction(TimedAction a){
        actions.add(a);
    }

    public void addAction(Action a, double delayINSECONDS){
        addAction(new TimedAction(a,delayINSECONDS));
    }
    /**Haven't figured out how to run loop automatically*/
    public void loop() {
        for (int i = 0; i < actions.size(); i++) {
            if (System.currentTimeMillis()/1000 - actions.get(i).startTime > actions.get(i).elapsedTime) {
                actions.get(i).a.action();
                actions.remove(i);
                i--; //step i back since we just removed what we were working on.
                // Interesting but useless side effect is we know how many actions we have in waiting
            }
        }
    }
}
