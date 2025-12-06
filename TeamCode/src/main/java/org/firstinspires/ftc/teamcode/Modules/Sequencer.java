package org.firstinspires.ftc.teamcode.Modules;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Placeholder;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;
import java.util.stream.Stream;

//practically copy-pasting, don't blame me -

@Placeholder(note = "Mostly complete, has not undergone peer review. Change this when peer reviewed.")
public class Sequencer { // Done by Owen
    public Map<String,ImpulseAction> impulseactions;

    public Map<String,LoopedAction> loopactions;
   // public Telemetry t = null;
    public enum SequenceType {
        IMPULSE,
        LOOPED
    }
    /**Creates a sequencer. This class allows you to perform tasks after a set amount of time.*/
    public Sequencer(){
        impulseactions = new HashMap<>();
        loopactions = new HashMap<>();

    }
   // ///only intended for use in SequencerTest. Not recommended.
   // public Sequencer(Telemetry t){
  //      impulseactions = new HashMap<>();
  //      loopactions = new HashMap<>();
  //      this.t = t;
  //  };

    public interface Action{
        void action();
    }

    /**One type of action that runs once after a delay, then ends.*/
    public static class ImpulseAction extends SeqAction{
        public ImpulseAction(Action a, double elapsedTime, String actionName) {
            super(a,elapsedTime,actionName);
        }
        public ImpulseAction(Action a, double elapsedTime) {
            super(a,elapsedTime, UUID.randomUUID().toString());
        }
    }
    /**Another type of action that runs forever after a delay, only stopping when killAction is set to true.*/
    public static class LoopedAction extends SeqAction {
        /**This determines if the action should be terminated. Set this to true and its action will be subsequently eliminated.*/
        public boolean killAction;
        public LoopedAction(Action a, double elapsedTime, String actionName) {
            super(a,elapsedTime,actionName);
            this.killAction = false;
        }
    }

    /**Retrieves a loop action based upon a name. Throws an exception if none is found.*/
    public LoopedAction getLoopAction(String name) {
        if (!loopactions.containsKey(name)) throw new RuntimeException("No loop action by the name of " + name + "!");
        return loopactions.get(name);
    }
    /**Retrieves an impulse action based upon a name. Throws an exception if none is found.*/
    public ImpulseAction getImpulseAction(String name) {
        if (!impulseactions.containsKey(name))
            throw new RuntimeException("No impulse action by the name of " + name + "!");
        return impulseactions.get(name);
    }
    /**A way to add actions with type as a parameter. Untested.*/
    public void addAction(Action a, SequenceType type, double elapsedTime, String actionName, Object... params) throws IllegalAccessException, InstantiationException, NoSuchMethodException, InvocationTargetException {
        switch (type) {
            case LOOPED:
                addLoopAction(a,elapsedTime,actionName);
            break;
            case IMPULSE:
                addImpulseAction(a,elapsedTime,actionName);
            break;
            default: throw new NullPointerException("Type likely null!");
        }
    }

    public void addImpulseAction(Action a, double delayINSECONDS, String name){
        addImpulseAction(new ImpulseAction(a,delayINSECONDS, name));

    }

    public void addImpulseAction(ImpulseAction a) {
        impulseactions.put(a.name,a);
    }


    public void addLoopAction(LoopedAction a) {
       // if (t != null) t.addData("actionNotNull",a != null);
        loopactions.put(a.name,a);
    }


    public void AddImpulseAction(Action a, double delayINSECONDS){
        addImpulseAction(new ImpulseAction(a,delayINSECONDS));
    }

    public void addLoopAction(Action a,double delayINSECONDS,String name) {
        addLoopAction(new LoopedAction(a,delayINSECONDS,name));
    }
    public void killLoopAction(String name, boolean kill) {
        if (!loopactions.containsKey(name)) throw new RuntimeException("No loop action of name: " + name + "!");
        loopactions.get("name").killAction = kill;
    }
    public void killImpulseAction(String name, boolean kill) {
        if (!loopactions.containsKey(name)) throw new RuntimeException("No impulse action of name: " + name + "!");
        loopactions.get("name").killAction = kill;
    }


    /**Process loop prototype. More untested than the impulse prototype. Mostly for debugging purposes, remove this and its if statement after peer review.*/
    public static final boolean runPrototype = true;

    //NOTE: dont change this to remove the actions to be killed within the primary loop, that throws a nice ConcurrentModificationException.
    public void loop() {
        List<String> remove = new ArrayList<>();

        for (Map.Entry<String,ImpulseAction> entry : impulseactions.entrySet()) {
            if ((double) System.currentTimeMillis() /1000 - entry.getValue().startTime > entry.getValue().elapsedTime) {
                entry.getValue().a.action();
                remove.add(entry.getKey());
            }
        }
        remove.forEach((string) -> impulseactions.remove(string));


        if (!runPrototype) return;


        remove.clear();
        for (Map.Entry<String,LoopedAction> act : loopactions.entrySet()) {
            boolean overrideAction = act.getValue().killAction;

            if (!overrideAction && (double) System.currentTimeMillis()/1000 - act.getValue().startTime > act.getValue().elapsedTime)
                act.getValue().a.action();

            else if (overrideAction) remove.add(act.getKey());
        }
        for (String s : remove) loopactions.remove(s);
    }
}
