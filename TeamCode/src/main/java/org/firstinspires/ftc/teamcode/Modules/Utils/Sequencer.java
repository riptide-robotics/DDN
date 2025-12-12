package org.firstinspires.ftc.teamcode.Modules.Utils;


import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;

import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.UUID;


/** 
 * The Sequencer takes snippets of code and marks them for execution in different ways. <br>
 * These are considered Actions. Together, the define what, when, and how a task is run. <br>
 * Actions that start with T, or Trip, act as a "tripwire." They run when the bot approaches a given location. <br>
 * If it does not, it runs after a given timer. <br>
 * An impulse action will run once. <br>
 * A loop action will run repeatedly, or until the action is set to be killed. <br> <br>
 * For now, the trip actions are not completed, and are completely untested. Please do not use them.
 **/
public class Sequencer { // Done by Owen
    public static final DistanceUnit unit = DistanceUnit.INCH;

    
    public Map<String,ImpulseAction> impulseactions;
    public Map<String, LoopAction> loopactions;
    public Map<String,TripImpulseAction> Timpulseactions;
    public Map<String,TripLoopAction> Tloopactions;
    @Nullable
    private final Drivetrain drive;



    private static final boolean processTripActions = false;




    // public Telemetry t = null;
    /**Outdated.*/
    public enum SequenceType {
        IMPULSE,
        LOOPED
    }
    /**Creates a sequencer. This class allows you to perform tasks after a set amount of time. <br>
     * A null drivetrain means that trip actions will not be triggered.
     * */
    public Sequencer(@Nullable Drivetrain drive){
        impulseactions = new HashMap<>();
        loopactions = new HashMap<>();
        Timpulseactions = new HashMap<>();
        Tloopactions = new HashMap<>();

        this.drive = drive;
    }
   // ///only intended for use in SequencerTest. Not recommended.
   // public Sequencer(Telemetry t){
  //      impulseactions = new HashMap<>();
  //      loopactions = new HashMap<>();
  //      this.t = t;
  //  };

    /**Exists as a way to store a snippet and nothing else.*/
    public interface Action{
        void action();
    }
    /**
     * This class exists to check if the bot is within one predetermined area. <br>
     * It will try to fix areas with impossible specifications. <br>
     * It will not attempt to fix areas that are out-of-bounds, to avoid this needing to be updated for a later game.
     * */
    public static class Area {
        /**It is recommended to use the setters to change these values - they'll correct some mistakes.*/
        public double xMin;
        /**It is recommended to use the setters to change these values - they'll correct some mistakes.*/
        public double yMin;
        /**It is recommended to use the setters to change these values - they'll correct some mistakes.*/
        public double xMax;
        /**It is recommended to use the setters to change these values - they'll correct some mistakes.*/
        public double yMax;
        
        public Area(double xMin, double yMin, double xMax, double yMax) {
            this.xMin = xMin;
            this.yMin = yMin;
            this.xMax = xMax;
            this.yMax = yMax;
            
            correctPositions();
        }
        /**Create a blank Area. This will never fire. */
        public Area() {
            this.xMin = -12000; //ensure the bot doesn't just fire off the second it is started by putting it one thousand feet away
            this.yMin = 0;
            this.xMax = -12000;
            this.yMax = 0;
        }
        /**This method attempts to fix broken areas with impossible positions, ensuring that they exist somewhere.*/
        private void correctPositions() {
            if (xMin > xMax) {
                double stored = xMin;
                xMin = xMax;
                xMax = stored;
            }
            if (yMin > yMax) {
                double stored = yMin;
                xMin = xMax;
                xMax = stored;
            }
        };
        /**Similar to setMaxPos, but reliant on the origin.*/
        public void setSize(double width, double height) {
            setMaxPos(xMin + width, yMin + height);
        }
        /**Identical to setMinPos, but shows which of the two systems are preferred here.*/
        public void setOrigin(double x, double y) {
            setMinPos(x,y);
        }
        /**Identical to setMinPos, but shows which of the two systems are preferred here.*/
        public void setOrigin(Pose2D pose) {
            setOrigin(pose.getX(Sequencer.unit),pose.getY(Sequencer.unit));
        }
        /**Identical to setMinPos, but shows which of the two systems are preferred here.*/
        public void setOrigin(EditablePose2D pose) {
            setOrigin(pose.getX(Sequencer.unit),pose.getY(Sequencer.unit));
        }
        public void setMinPos(double x, double y) {
            this.xMin = x;
            this.yMin = y;
            
            correctPositions();
        }
        public void setMaxPos(double x, double y) {
            this.xMax = x;
            this.yMax = y;
            
            correctPositions();
        }
        public void setMinPos(Pose2D pose) {
            setMinPos(pose.getX(Sequencer.unit),pose.getY(Sequencer.unit));
        }
        public void setMaxPos(Pose2D pose) {
            setMaxPos(pose.getX(Sequencer.unit),pose.getY(Sequencer.unit));
        }
        public void setMinPos(EditablePose2D pose) {
            setMinPos(pose.getX(Sequencer.unit),pose.getY(Sequencer.unit));
        }
        public void setMaxPos(EditablePose2D pose) {
            setMaxPos(pose.getX(Sequencer.unit),pose.getY(Sequencer.unit));
        }
        /**
         * Check if the bot is within the area defined.
         * @param drive The drivetrain that is used to check. Looking for a way to remove this.
         * */
        public boolean botWithinArea(Drivetrain drive) {
            if (drive == null) return false;

            double X = drive.getPinpoint().getPosX(Sequencer.unit);
            double Y = drive.getPinpoint().getPosY(Sequencer.unit);

            //CONSISTENCY
            if (X > xMax || X < xMin) return false;
            if (Y > yMax || Y < yMin) return false;

            return true;
        }
    }

    /**One type of action that runs once after a delay, then ends.*/
    public static class ImpulseAction extends SeqAction {
        public ImpulseAction(Action a, double elapsedTime, String actionName) {
            super(a,elapsedTime,actionName);
        }
        public ImpulseAction(Action a, double elapsedTime) {
            super(a,elapsedTime, UUID.randomUUID().toString());
        }
    }
    /**Another type of action that runs forever after a delay, only stopping when killAction is set to true.*/
    public static class LoopAction extends SeqAction {
        /**This determines if the action should be terminated. Set this to true and its action will be subsequently eliminated.*/
        public boolean killAction;

        public LoopAction(Action a, double elapsedTime, String actionName) {
            super(a,elapsedTime,actionName);
            this.killAction = false;
        }
    }
    public static class TripImpulseAction extends SeqAction {
        public Area area;
        TripImpulseAction(Action a, Area area, String name) {
            super(a, 0, name);
            this.area = area;
        }
    }
    public static class TripLoopAction extends SeqAction {
        public Area area;
        public boolean killAction = false;
        TripLoopAction(Action a, Area area, String name) {
            super(a, 0, name);
            this.area = area;
        }
    }

    /**A way to add actions with type as a parameter. Untested, outdated.*/
    @Deprecated
    public void addAction(Action a, SequenceType type, double elapsedTime, String actionName, Object... params) throws IllegalAccessException, InstantiationException, NoSuchMethodException, InvocationTargetException {
        switch (type) {
            case LOOPED:
                addLoopAction(a,elapsedTime,actionName);
            break;
            case IMPULSE:
                addImpulseAction(a,elapsedTime,actionName);
            break;
            default: throw new IllegalArgumentException("Type likely null!");
        }
    }
    
    //NOTE: don't change this to remove the actions to be killed within the primary loop, that throws a nice ConcurrentModificationException.
    
    /*
    For anyone who wants to add or modify a type of action, the following may be helpful.

    The following example is of ImpulseAction.

        remove = processImpulseActions();
        for (String s : remove) impulseactions.remove(s);
    
    The first line processes all actions. The returned value gives the actions to be removed.
    The second line removes those actions.
    
    The 'remove' is only initialized once so you don't have to see a bunch of different variables with almost identical purposes.
    */

    /**
     *  Process all actions queued up. <br>
     *  This handles all the checks necessary; <br>
     *  all that is needed is to put this in a loop that can be run.
     *  */
    public void loop() {
        List<String> remove;
        
        
        remove = processImpulseActions();
        for (String s : remove) impulseactions.remove(s);

        remove = processLoopActions();
        for (String s : remove) loopactions.remove(s);

        remove = processTImpulseActions();
        for (String s : remove) Timpulseactions.remove(s);
        
        remove = processTLoopActions();
        for (String s : remove) Tloopactions.remove(s);

    }
    /**
     * Internal helper method to process specifically Impulse Actions. <br>
     * THIS WILL NOT REMOVE THE ACTION ON ITS OWN!
     * @return The names of all actions that are to be removed. 
     * */
    private List<String> processImpulseActions() {
        List<String> remove = new ArrayList<>();
        
        for (Map.Entry<String,ImpulseAction> entry : impulseactions.entrySet()) {
            if ((double) System.currentTimeMillis() /1000 - entry.getValue().startTime > entry.getValue().elapsedTime) {
                entry.getValue().a.action();
                remove.add(entry.getKey());
            }
        }
        return remove;
    }
    /**
     * Internal helper method to process specifically Loop Actions. <br>
     * THIS WILL NOT REMOVE THE ACTION ON ITS OWN!
     * @return The names of all actions that are to be removed. 
     **/
    private List<String> processLoopActions() {
        List<String> remove = new ArrayList<>();
        
        for (Map.Entry<String, LoopAction> act : loopactions.entrySet()) {
            boolean overrideAction = act.getValue().killAction;

            if (!overrideAction && (double) System.currentTimeMillis()/1000 - act.getValue().startTime > act.getValue().elapsedTime)
                act.getValue().a.action();

            else if (overrideAction) remove.add(act.getKey());
        }
        return remove;
    }
    /**
     * Internal helper method to process specifically Trip Impulse Actions. <br>
     * THIS WILL NOT REMOVE THE ACTION ON ITS OWN!
     * @return The names of all actions that are to be removed. 
     * */
    private List<String> processTImpulseActions() {
        List<String> remove = new ArrayList<>();

        for (Map.Entry<String, TripImpulseAction> act : Timpulseactions.entrySet()) {
            if (act.getValue().area.botWithinArea(this.drive)) {
                act.getValue().a.action();
                remove.add(act.getKey());
            }
        }
        return remove;
    }
    /**
     * Internal helper method to process specifically Trip Loop Actions. <br>
     * THIS WILL NOT REMOVE THE ACTION ON ITS OWN!
     * @return The names of all actions that are to be removed. 
     * */
    private List<String> processTLoopActions() {
        List<String> remove = new ArrayList<>();

        for (Map.Entry<String,TripLoopAction> act : Tloopactions.entrySet()) {
            boolean overrideAction = act.getValue().killAction;

            if (!overrideAction && act.getValue().area.botWithinArea(this.drive))
                act.getValue().a.action();

            else if (overrideAction) remove.add(act.getKey());
        }
        return remove;
    }

    
    /**Retrieves an impulse action based upon a name. Throws an exception if none is found.*/
    public ImpulseAction getImpulseAction(String name) {
        if (!impulseactions.containsKey(name))
            throw new RuntimeException("No impulse action by the name of " + name + "!");
        return impulseactions.get(name);
    }
    
    /**Retrieves a loop action based upon a name. Throws an exception if none is found.*/
    public LoopAction getLoopAction(String name) {
        if (!loopactions.containsKey(name))
            throw new RuntimeException("No loop action by the name of " + name + "!");
        return loopactions.get(name);
    }
    
    /**Retrieves a trip impulse action based upon a name. Throws an exception if none is found.*/
    public TripImpulseAction getTImpulseAction(String name) {
        if (!Timpulseactions.containsKey(name))
            throw new RuntimeException("No trip impulse action by the name of " + name + "!");
        return Timpulseactions.get(name);
    }
    
    /**Retrieves a trip loop action based upon a name. Throws an exception if none is found.*/
    public TripLoopAction getTLoopAction(String name) {
        if (!Tloopactions.containsKey(name))
            throw new RuntimeException("No trip loop action by the name of " + name + "!");
        return Tloopactions.get(name);
    }
    
    public void addImpulseAction(ImpulseAction a) {
        impulseactions.put(a.name,a);
    }

    public void addLoopAction(LoopAction a) {
        loopactions.put(a.name,a);
    }

    public void addTImpulseAction(TripImpulseAction a) {
        Timpulseactions.put(a.name,a);
    }

    public void addTLoopAction(TripLoopAction a) {
        Tloopactions.put(a.name,a);
    }

    public void addImpulseAction(Action a, double delayINSECONDS){
        addImpulseAction(new ImpulseAction(a,delayINSECONDS));
    }

    public void addImpulseAction(Action a, double delayINSECONDS, String name){
        addImpulseAction(new ImpulseAction(a,delayINSECONDS, name));

    }
    public void addLoopAction(Action a,double delayINSECONDS, String name) {
        addLoopAction(new LoopAction(a,delayINSECONDS,name));
    }

    public void addTImpulseAction(Action a, Area area, String name) {
        addTImpulseAction(new TripImpulseAction(a,area,name));
    }

    public void addTLoopAction(Action a, Area area, String name) {
        addTLoopAction(new TripLoopAction(a,area,name));
    }

    public void killLoopAction(String name, boolean kill) {
        if (!loopactions.containsKey(name)) throw new NoSuchElementException("No loop action of name: " + name + "! Make sure this is created before you delete it.");
        loopactions.get(name).killAction = kill;
    }
    public void killTLoopAction(String name, boolean kill) {
        if (!Tloopactions.containsKey(name)) throw new NoSuchElementException("No loop action of name: " + name + "! Make sure this is created before you delete it.");
        Tloopactions.get(name).killAction = kill;
    }
}
