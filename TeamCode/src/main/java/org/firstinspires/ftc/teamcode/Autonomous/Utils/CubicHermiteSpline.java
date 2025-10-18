package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;

public class CubicHermiteSpline {

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

    public static class SplineSegment{
        double xCubedCoefficient;
        double xSquaredCoefficient;
        double xLinearCoefficient;
        double xConstantCoefficient;

        double yCubedCoefficient;
        double ySquaredCoefficient;
        double yLinearCoefficient;
        double yConstantCoefficient;

        double segmentTime;

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

        public double getX(double dt){
            double checkeddt = Math.min(dt, segmentTime);
            double frac = dt/segmentTime;
            return xCubedCoefficient * Math.pow(frac, 3) + xSquaredCoefficient * Math.pow(frac, 2) + xLinearCoefficient * frac + xConstantCoefficient;
        }

        public double getY(double dt){
            double checkeddt = Math.min(dt, segmentTime);
            double frac = dt/segmentTime;
            return yCubedCoefficient * Math.pow(frac, 3) + ySquaredCoefficient * Math.pow(frac, 2) + yLinearCoefficient * frac + yConstantCoefficient;
        }


    }

}
