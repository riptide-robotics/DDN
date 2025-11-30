package org.firstinspires.ftc.teamcode.Modules;

import android.provider.ContactsContract;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Placeholder;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.UUID;

//practically copy-pasting, don't blame me -

@Placeholder(note = "mostly functioning but not tested")
public class Sequencer { // Done by Owen
    public Map<String,ImpulseAction> impulseactions;
    public Map<String,LoopedAction> loopactions;
   // public Telemetry t = null;
    public enum SequenceType {
        IMPULSE,
        LOOPED
    }
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

    public static class ImpulseAction extends SeqAction{
        public ImpulseAction(Action a, double elapsedTime, String actionName) {
            super(a,elapsedTime,actionName);
        }
        public ImpulseAction(Action a, double elapsedTime) {
            super(a,elapsedTime, UUID.randomUUID().toString());
        }
    }
    public static class LoopedAction extends SeqAction {
        public boolean killAction;
        public LoopedAction(Action a, double elapsedTime, String actionName) {
            super(a,elapsedTime,actionName);
            this.killAction = false;
        }
    }

    public LoopedAction getLoopAction(String name) {
        if (!loopactions.containsKey(name)) throw new RuntimeException("No loop action by the name of " + name + "!");
        return loopactions.get(name);
    }
    public ImpulseAction getImpulseAction(String name) {
        if (!impulseactions.containsKey(name)) throw new RuntimeException("No impulse action by the name of " + name + "!");
        return impulseactions.get(name);
    }

    public void addAction(Action a, SequenceType type, double elapsedTime, String actionName, Object... params) throws IllegalAccessException, InstantiationException, NoSuchMethodException, InvocationTargetException {
        switch (type) {
            case LOOPED: {
                addLoopAction(a,elapsedTime,actionName);
            }
            case IMPULSE: {
                addImpulseAction(a,elapsedTime,actionName);
            }
            default: throw new NullPointerException("Type almost certainly null!");
        }
    }

    public void addImpulseAction(Action a, double delayINSECONDS, String name){
        addImpulseAction(new ImpulseAction(a,delayINSECONDS, name));
    }

    //A:one action
    //B:concurrent events set outtakePID speed to something
    //C:testing multiple sequences in order
    //D:test moving multiple things in order



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
    /**working on something else right now; dont change this unless you are extremely confident in this working <br>
    (which you shouldn't be) <br>
     Currently active, DO NOT USE THE SEQUENCER!
     */
    public static final boolean runPrototype = true;

    /**Haven't figured out how to run loop automatically. Also, has extremely untested prototype code.*/
    public void loop() {
        List<String> remove = new ArrayList<>();

        for (Map.Entry<String,ImpulseAction> entry : impulseactions.entrySet()) {
          //  if (t != null) t.addData("finalloc",entry.getValue() != null);
            if ((double) System.currentTimeMillis() /1000 - entry.getValue().startTime > entry.getValue().elapsedTime) {
                entry.getValue().a.action();
                remove.add(entry.getKey());
            }
        }
        remove.forEach((string) -> impulseactions.remove(string));


        if (!runPrototype) return;

        //NOTE: dont change this to remove the actions to be killed within the primary loop, that theows a nice ConcurrentModificationException.

        //create list for actions that are to be removed
        //handle looped actions
        remove.clear();
        for (Map.Entry<String,LoopedAction> act : loopactions.entrySet()) {
            boolean overrideAction = act.getValue().killAction;

            //messy if block
            if (!overrideAction && (double) System.currentTimeMillis()/1000 - act.getValue().startTime > act.getValue().elapsedTime)
               //run the action
                act.getValue().a.action();

            //add actions to be removed to the list
            else if (overrideAction) remove.add(act.getKey());
        }
        //kill actions on the hit list
        for (String s : remove) loopactions.remove(s);
    }
}
