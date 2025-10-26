package org.firstinspires.ftc.teamcode.Autonomous.Utils.Actions;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

// Courtesy of Aaron Xie, Oct 21, 2025
public class TimedActionController {

    public interface Action {
        void action();
    }

    private ArrayList<TimedAction> actionList = new ArrayList<>();
    private final ElapsedTime timer;

    public TimedActionController(ArrayList<TimedAction> actionList, ElapsedTime timer) {
        this.actionList = actionList;
        this.timer = timer;
    }

    public void step() {
        double currTime = timer.seconds();

        for(TimedAction ta : actionList){
            if(ta.isFinished(currTime)){
                if(!ta.hasCleanedUp()) ta.cleanup();
                continue;
            }
            if(!ta.isReady(currTime)){continue;}
            ta.action();
        }
    }

    public static class TimedAction {
        private final double startTimeInSeconds;
        private final double endTimeInSeconds;

        private final Action action;
        private boolean actionedOnce = false;
        private Action runOnce = () -> {
        };

        private Action cleanup = () -> {
        };
        private boolean cleanedUpOnce = false;

        public TimedAction(double startTimeInSeconds, double endTimeInSeconds, Action action, Action runOnce, Action cleanup) {
            if(endTimeInSeconds < startTimeInSeconds){throw new IllegalArgumentException("end time cannot be smaller than start time");}
            this.startTimeInSeconds = startTimeInSeconds;
            this.endTimeInSeconds = endTimeInSeconds;
            this.action = action;
            this.cleanup = cleanup;
            this.runOnce = runOnce;
        }

        //someone can make a builder method for this I can't be bothered
        public TimedAction(double startTimeInSeconds, double endTimeInSeconds, Action action, Action cleanup, boolean cleanupAction) {
            this(startTimeInSeconds, endTimeInSeconds, action, () -> {}, cleanup);
        }

        public TimedAction(double startTimeInSeconds, double endTimeInSeconds, Action action, Action runOnce) {
            this(startTimeInSeconds, endTimeInSeconds, action, runOnce, () -> {});
        }

        public TimedAction(double startTimeInSeconds, double endTimeInSeconds, Action action) {
            this(startTimeInSeconds, endTimeInSeconds, action, () -> {}, () -> {});
        }

        public boolean isReady(double time) {
            return time >= startTimeInSeconds && time < endTimeInSeconds;
        }

        public boolean isFinished(double time) {
            return endTimeInSeconds < time;
        }

        public void action() {
            if (!actionedOnce) {
                actionedOnce = true;
                runOnce.action();
            }
            action.action();
        }

        public void cleanup() {
            if (cleanedUpOnce) {
                return;
            }
            cleanup.action();
            cleanedUpOnce = true;
        }

        public boolean hasActioned(){return actionedOnce;}
        public boolean hasCleanedUp(){return cleanedUpOnce;}
    }

    public static class TimedActionBuilder {
        private final ArrayList<TimedAction> actionList = new ArrayList<>();
        private final ElapsedTime timer;

        public TimedActionBuilder addTimedAction(TimedAction a) {
            actionList.add(a);
            return this;
        }

        public TimedActionBuilder(ElapsedTime timer) {
            this.timer = timer;
        }

        public TimedActionController build() {
            return new TimedActionController(actionList, timer);
        }
    }
}
