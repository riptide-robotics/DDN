package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.ArrayList;

//Know that these are hermite splines
public class CubicSplinePath {

    public static class CubicPathPoint {
        private double x;
        private double y;
        private double heading;
    }

    public static class CubicSplineBuilder{
        private ArrayList<CubicSplinePath.CubicPathPoint> path = new ArrayList<>();


        public Path.PathBuilder addNewFullPoint(Waypoint w, Path.RobotAction a, double delay){
            Path.PathPoint p = new Path.PathPoint(w, a, delay);
            path.add(p);
            return this;
            Pose2D
        }

        public ArrayList<Path.PathPoint> getPath(){
            return path;
        }

        public Path build(){
            return new Path(this);
        }

    }
}
