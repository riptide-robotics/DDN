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
 * A pro of having the motion profile litteraly be a linear pass-through of each spline segment is that getPosition is actually very easy to calculate, and we can use getDelayUntilNextPoint find the scale size.
 * This will not be true for other trajectory builders, however, because those will probably have curvature dependent motion profiles.
 * I made this without knowing how to derive C2 Continuous splines, so this spline does not have continous acceleration at knot points
 * I don't know if we can implement some velocity based on curvature at all in these points. This would basically mean a total refactor, because once we add in curvature control, we can't specifically denote what time we want the robot to make (I think idk some math is required). But if it's possible without a total refactor, then go for it.
 */
public class CubicHermiteSpline extends Trajectory {

    private Path points;
    private ArrayList<SplineSegment> spline;
    private double totalTime = 0;

    public CubicHermiteSpline(Path path) {
        this.points = path;
        if (path.getPathSize() < 2){ throw new IllegalArgumentException("path length needs to be greater than 1");}
        ArrayList<SplineSegment> spline  = new ArrayList<>();
        for (int i = 0; i<path.getPathSize() - 1; i++){
            spline.add(new SplineSegment(path.get(i), path.get(i+1), path.get(i).getDelayUntilNextPoint()));
            totalTime += path.get(i).getDelayUntilNextPoint();
        }
        totalTime += path.get(path.getPathSize()-1).getDelayUntilNextPoint();
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

    /**
     * returns how many spline segments are in this one trajectory
     * @return the length of the array spline
     */
    public int getSplineLength(){
        return spline.size();
    }

    /**
     * returns the a spline segment of the trajectory at index i
     * @param i index of the path
     * @return the specific spline segment associated with the index
     */
    public SplineSegment getSegment(int i){
        return spline.get(i);
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
        public final double totalArcLength = getArcLengthTo3DecimalPlaces();

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

            //bad practice??
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

        //integrate speed over time to get distance
        //trapezoidal approximation
        public double getArcLength(int intervals){
            if(intervals <= 1){throw new IllegalArgumentException("Arc Length Sample count must be greater than 2");}
            double distance = 0;
            double step = segmentTime/intervals;
            double prev = getSpeed(0);
            for(int i = 0; i < intervals; i++){
                double curr = getSpeed(i * step);
                distance += 0.5 * (curr + prev) * step;
                prev = curr;
            }
            return distance;
        }

        /**
         * will recusively run until stable to 3 decimal places, or until we've done a maximum number of
         * allowed intervals
         */
        public double getArcLengthTo3DecimalPlaces(){
            final double TOLERANCE = 5e-4;   // 0.0005 guarantees stability at 3 d.p. I think
            final int MIN_INTERVALS = 32;
            final int MAX_INTERVALS = 1 << 14;

            int n = MIN_INTERVALS;
            double sN = getArcLength(n);

            while (n < MAX_INTERVALS) {
                int doubleN = n * 2;
                double sN2 = getArcLength(doubleN);
                if (Math.abs(sN2 - sN) < TOLERANCE) {
                    return sN2;
                }
                sN = sN2;
                n = doubleN;
            }
            // Best effort under the cap
            return sN;
        }

        public double getDistanceTraveled(double pastTime, double currTime){
            double t = Math.min(currTime, segmentTime);
            double dist = 0;

            double dt = t - pastTime;
            double currSpeed = getSpeed(currTime);
            double pastSpeed = getSpeed(pastTime);

            if (dt > 0){
               dist += 0.5 * (currSpeed + pastSpeed) * dt;
            }
            return dist;
        }

        public double getDistanceTraveled(int intervals, double currTime) {
                        if (intervals < 1) {
                throw new IllegalArgumentException("intervals must be >= 1");
            }
                        if (currTime <= 0.0) return 0.0;
                double T = Math.min(currTime, segmentTime);

            double step = T / intervals;
                double dist = 0.0;
            double prev = getSpeed(0.0);

                for (int i = 1; i <= intervals; i++) {
                        double t = i * step;
                double curr = getSpeed(t);
                dist += 0.5 * (prev + curr) * step;
                prev = curr;
            }
            return dist;
        }
    }

}
