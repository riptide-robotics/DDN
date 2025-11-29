package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Placeholder;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

//practically copy-pasting, don't blame me -

@Placeholder(note = "mostly functioning but not tested")
public class Sequencer {
    public ArrayList<TimedAction> actions;
    public Map<String,LoopedAction> loopactions;
    public Sequencer(){
        actions = new ArrayList<>();
        loopactions = new HashMap<>();
    };

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
    public static class LoopedAction {
        Action a;
        boolean killAction = false;
        final double startTime;
        double elapsedTime;
        String actionName;
        public LoopedAction(Action a, double delay, String actionName) {
            this.a = a;
            this.elapsedTime = delay;
            this.startTime = (double) System.currentTimeMillis() /1000;
            this.actionName = actionName;
        }
    }


    //A:one action
    //B:concurrent events set outtakePID speed to something
    //C:testing multiple sequences in order
    //D:test moving multiple things in order


    public void addAction(TimedAction a) {
        actions.add(a);
    }

    public void addLoopAction(LoopedAction a) {
        loopactions.put(a.actionName,a);
    }


    public void addAction(Action a, double delayINSECONDS){
        addAction(new TimedAction(a,delayINSECONDS));
    }
    public void addLoopAction(Action a,double delayINSECONDS,String name) {
        addLoopAction(new LoopedAction(a,delayINSECONDS,name));
    }
    /**working on something else right now; dont change this unless you are extremely confident in this working <br>
    (which you shouldn't be) */
    public static final boolean runPrototype = false;

    /**Haven't figured out how to run loop automatically. Also, has extremely untested prototype code.*/
    public void loop() {
        for (int i = 0; i < actions.size(); i++) {
            if ((double) System.currentTimeMillis() /1000 - actions.get(i).startTime > actions.get(i).elapsedTime) {
                actions.get(i).a.action();
                actions.remove(i);
                i--; //step i back since we just removed what we were working on.
                // Interesting but useless side effect is we know how many actions we have in waiting
            }
        }
        if (!runPrototype) return;

        //NOTE: dont change this to remove the actions to be killed within the primary loop, that theows a nice ConcurrentModificationException.

        //create list for actions that are to be removed
        List<String> remove = new ArrayList<>();
        //handle looped actions
        for (Map.Entry<String,LoopedAction> act : loopactions.entrySet()) {
            boolean overrideAction = act.getValue().killAction;

            //messy if block
            if (  //if the action is queued to be removed dont run it
                    !overrideAction &&
                  //check the timer and make sure you can run it
                    (double) System.currentTimeMillis()/1000 - act.getValue().startTime
                            > act.getValue().elapsedTime)
            {   //run the action
                act.getValue().a.action();
            }
            //add actions to be removed to the list
            if (overrideAction) remove.add(act.getKey());
        }
        //kill actions on the hit list
        for (String s : remove) loopactions.remove(s);
    }
    public void teleloop(Telemetry t) {
        t.addData("Action-ACTION_LIST_SIZE", actions.size());
        for (int i = 0; i < actions.size(); i++) {
            t.addData("Action-C1",(double) System.currentTimeMillis() /1000 - actions.get(i).startTime);
            t.addData("Action-C2",actions.get(i).elapsedTime);
            if ((double) System.currentTimeMillis() /1000 - actions.get(i).startTime > actions.get(i).elapsedTime) {
                t.addData("Action-ACTION-LAST-EXECUTED",true);
                actions.get(i).a.action();
                actions.remove(i);
                i--; //step i back since we just removed what we were working on.
                // Interesting but useless side effect is we know how many actions we have in waiting
            }
        }
    }
}
