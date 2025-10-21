package org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Path;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

import java.util.ArrayList;

/**
 * Given a set of points, interpolates a trajectory to follow.
 * A trajectory is a path with some kind of motion constraint/profile
 * Does not have continuous Acceleration across knot points
 * The profile is a really bad one, it's simply a one to one mapping of pose to time.
 * That's why there isn't a dedicated Profile Class to calculate everything like there is in the linear case.
 * Use this graph to visualize connections:
 * https://www.desmos.com/calculator/blzsusrpwd
 *
 * Technical Details:
 * I made this without knowing how to derive C2 Continuous splines, so this spline does not have continous acceleration at knot points
 * I don't know if we can implement some velocity based on curvature at all in these points. This would basically mean a total refactor, because once we add in curvature control, we can't specifically denote what time we want the robot to make (I think idk some math is required). But if it's possible without a total refactor, then go for it.
 */
public class CubicHermiteSpline extends Trajectory {

    private Path points;
    private ArrayList<SplineSegment> spline;

    public CubicHermiteSpline(Path path) {
        this.points = path;
        if (path.getPathSize() < 2){ throw new IllegalArgumentException("path length needs to be greater than 1");}
        ArrayList<SplineSegment> spline  = new ArrayList<>();
        for (int i = 0; i<path.getPathSize() - 1; i++){
            spline.add(new SplineSegment(path.get(i), path.get(i+1), path.get(i).getDelayUntilNextPoint()));
        }
        this.spline = spline;
    }

    /**
     *  Given the current time, traverses the Array list of SplineSegments to retrieve the correct location
     * @param dt current time in seconds
     * @return the expected location at time dt
     */
    public EditablePose2D getPosition(double dt){
        double elapsedTime = 0;
        for(int i = 0; i < spline.size(); i++){
            if (elapsedTime + spline.get(i).segmentTime > 0){
                return new EditablePose2D(
                        spline.get(i).getX(dt-elapsedTime),
                        spline.get(i).getY(dt-elapsedTime),
                        spline.get(i).getTangentHeading(dt-elapsedTime),
                        DistanceUnit.INCH
                );
            }
            elapsedTime += spline.get(i).segmentTime;
        }

        return new EditablePose2D(
                spline.get(spline.size()-1).getX(dt),
                spline.get(spline.size()-1).getY(dt),
                spline.get(spline.size()-1).getTangentHeading(dt),
                DistanceUnit.INCH);
    }

    public static class SplineSegment{
        private final double xCubedCoefficient;
        private final double xSquaredCoefficient;
        private final double xLinearCoefficient;
        private final double xConstantCoefficient;

        private final double yCubedCoefficient;
        private final double ySquaredCoefficient;
        private final double yLinearCoefficient;
        private final double yConstantCoefficient;

        public final double segmentTime;

        public SplineSegment(Path.PathPoint p0, Path.PathPoint p1, double segmentTime) {
            double x0 = p0.getPos().getX(DistanceUnit.INCH);
            double y0 = p0.getPos().getY(DistanceUnit.INCH);
            double dx0 = p0.getGoalVelocityInInPerSec() * Math.cos(p0.getHeading()) * segmentTime;
            double dy0 = p0.getGoalVelocityInInPerSec() * Math.sin(p0.getHeading()) * segmentTime;

            double x1 = p1.getPos().getX(DistanceUnit.INCH);
            double y1 = p1.getPos().getY(DistanceUnit.INCH);
            double dx1 = p1.getGoalVelocityInInPerSec() * Math.cos(p1.getHeading()) * segmentTime;
            double dy1 = p1.getGoalVelocityInInPerSec() * Math.sin(p1.getHeading()) * segmentTime;

            this.xCubedCoefficient = 2 * x0 + dx0 - 2 * x1 + dx1;
            this.xSquaredCoefficient = -3 * x0 - 2 * dx0 + 3 * x1 - dx1;
            this.xLinearCoefficient = dx0;
            this.xConstantCoefficient = x0;

            this.yCubedCoefficient = 2 * y0 + dy0 - 2 * y1 + dy1;
            this.ySquaredCoefficient = -3 * y0 - 2 * dy0 + 3 * y1 - dy1;
            this.yLinearCoefficient = dy0;
            this.yConstantCoefficient = y0;

            this.segmentTime = segmentTime;
        }

        public double s(double dt){
            if(dt < 0) { throw new IllegalArgumentException("Uhh, time is negative idk how you did it"); }
            double checkeddt = Math.min(dt, segmentTime);
            return checkeddt/segmentTime;
        }

        public double getX(double dt){
            double frac = s(dt);
            return xCubedCoefficient * Math.pow(frac, 3) + xSquaredCoefficient * Math.pow(frac, 2) + xLinearCoefficient * frac + xConstantCoefficient;
        }

        public double getY(double dt){
            double frac =s(dt);
            return yCubedCoefficient * Math.pow(frac, 3) + ySquaredCoefficient * Math.pow(frac, 2) + yLinearCoefficient * frac + yConstantCoefficient;
        }

        public double getXPrime(double dt){
           double frac = s(dt);
           double dXdS = 3*xCubedCoefficient*Math.pow(frac, 2) + 2*xSquaredCoefficient * frac + xLinearCoefficient;
           return dXdS/segmentTime;
        }

        public double getYPrime(double dt){
            double frac = s(dt);
            double dYdS =  3*yCubedCoefficient*Math.pow(frac, 2) + 2*ySquaredCoefficient * frac + yLinearCoefficient;
            return dYdS/segmentTime;
        }

        //Defaulting to call this speed now, because we are dealing with vectors and velocity is now a vector

        /**
         * magnitude of the tangent vector at a point
         * @param dt elapsed time, should be between 0 and segmentTime
         * @return a double that represents the magnitude of the tangent vector of the curve
         */
        public double getSpeed(double dt){
            double dx = getXPrime(dt);
            double dy = getYPrime(dt);
            return Math.hypot(dx, dy);
        }

        // radians
        public double getTangentHeading(double dt){
            double dx = getXPrime(dt);
            double dy = getYPrime(dt);
            return Math.atan2(dy, dx);
        }



    }

}
