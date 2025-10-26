package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_A_VERT;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V_VERT;

import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

import java.util.ArrayList;

// maybe usable, idk  usable, needs some edits.

public class Path {
    private ArrayList<PathPoint> path;

    public enum HeadingBehavior {
        HEAD_TANGENT,
        HEAD_FLOAT
    }

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

        private EditablePose2D pos;
        private double heading;
        private double goalVelocity;
        private double goalAcceleration;
        private double delayUntilNextPoint = 0;
        private HeadingBehavior headingBehavior = HeadingBehavior.HEAD_TANGENT;


        public PathPoint(EditablePose2D position, double goalVelocity, double goalAcceleration, double delayUntilNextPoint, HeadingBehavior headingBehavior){
            this.pos = position;
            this.headingBehavior = headingBehavior;
            this.delayUntilNextPoint = delayUntilNextPoint;
            this.goalVelocity = goalVelocity; // velocity magnitude, direction will be the heading.
            this.goalAcceleration = goalAcceleration;
            this.heading = position.getH();
        }

        public PathPoint(EditablePose2D position, double delayUntilNextPoint){
            this(position, MAX_V_VERT, MAX_A_VERT, delayUntilNextPoint, HeadingBehavior.HEAD_TANGENT);
        }

        public PathPoint(EditablePose2D position, double goalVelocity, double delayUntilNextPoint){
            this(position, goalVelocity, 1, delayUntilNextPoint, HeadingBehavior.HEAD_TANGENT );
        }

        public double getDelayUntilNextPoint(){
            return delayUntilNextPoint;
        }

        public double getGoalVelocityInInPerSec() {return goalVelocity;}

        public double getGoalAcceleration() {return goalAcceleration;}

        public EditablePose2D getPos(){
            return pos;
        }

        public double getHeading(){
            return heading;
        }
    }

    public static class PathBuilder{
        private ArrayList<PathPoint> path = new ArrayList<PathPoint>();

        public PathBuilder addNewPoint(EditablePose2D position, double delay){
            PathPoint p = new PathPoint(position, delay);
            path.add(p);
            return this;
        }

        public PathBuilder addNewPoint(EditablePose2D position, double goalVelocity, double goalAcceleration, double delay, FollowMethod followMethod){
            PathPoint p = new PathPoint(position, goalVelocity, goalAcceleration, delay, followMethod);
            path.add(p);
            return this;
        }

        public PathBuilder addNewSplinePoint(EditablePose2D position, double goalVelocity, double delayUntilNextPoint){
            PathPoint p = new PathPoint(position, goalVelocity, delayUntilNextPoint);
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
