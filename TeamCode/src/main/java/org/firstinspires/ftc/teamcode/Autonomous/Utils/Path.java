package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_A_VERT;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V_VERT;

import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

import java.util.ArrayList;

// maybe usable, idk  usable, needs some edits.

public class Path {
    public enum FollowMethod {
        HEAD_FORWARD,
        HEAD_TANGENT, // runs tangent to path,
        HEAD_FLOAT
    }

    public interface RobotAction {
        void Action();
    }

    private ArrayList<PathPoint> path = new ArrayList<PathPoint>();

    public Path(PathBuilder b){
        this.path = b.getPath();
    }

    /**
     * @param i index of the waypoint list
     * @return a pathpoint at index i
     */
    public PathPoint get(int i){
        return path.get(i);
    }

    public int getPathSize(){
        return path.size();
    }

    public static class PathPoint{

        private RobotAction a = null;
        private EditablePose2D pos;
        private double heading;
        private double goalVelocity;
        private double goalAcceleration;
        private double delayUntilNextPoint = 0;
        private FollowMethod followMethod = FollowMethod.HEAD_TANGENT;

        public PathPoint(EditablePose2D position, double goalVelocity, double goalAcceleration, RobotAction a, double delayUntilNextPoint, FollowMethod followMethod){
            this.a = a;
            this.pos = position;
            this.followMethod = followMethod;
            this.delayUntilNextPoint = delayUntilNextPoint;
            this.goalVelocity = goalVelocity; // velocity magnitude, magnitude will be the heading.
            this.goalAcceleration = goalAcceleration;
        }

        public PathPoint(EditablePose2D position, RobotAction a, double delayUntilNextPoint){
            this(position, MAX_V_VERT, MAX_A_VERT, a, delayUntilNextPoint, FollowMethod.HEAD_TANGENT);
        }

        public RobotAction getAction(){
            return this.a;
        }

        public double getDelayUntilNextPoint(){
            return delayUntilNextPoint;
        }

        public double getGoalVelocity() {return goalVelocity;}

        public double getGoalAcceleration() {return goalAcceleration;}

        public EditablePose2D getLocation(){
            return pos;
        }
    }

    public static class PathBuilder{
        private ArrayList<PathPoint> path = new ArrayList<PathPoint>();

        // This is the grave of a few PathBuilders that nobody knows why they existed

        public PathBuilder addNewFullPoint(EditablePose2D position, RobotAction a, double delay){
            PathPoint p = new PathPoint(position, a, delay);
            path.add(p);
            return this;
        }

        public ArrayList<PathPoint> getPath(){
            return path;
        }

        public Path build(){
            return new Path(this);
        }

    }
}
