package org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories;

import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_A_VERT;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V;
import static org.firstinspires.ftc.teamcode.riptideUtil.MAX_V_VERT;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.Pair;

import java.util.ArrayList;

// This is for tank drive. There is no "free heading interpolation" in this mode. The heading will always be set to the direction of the path points.
// if you need help expanding to holonomic drive trains, you can always hit me up
// --Aaron Xie
public class LinearTrajectoryBuilder {
    ArrayList<Pose2D> controlPoints;
    ArrayList<Pair<Integer, Double>> speedScalars; // To scale the speed of a certain segment

    public LinearTrajectoryBuilder(ArrayList<Pose2D> controlPoints, ArrayList<Pair<Integer, Double>> speedScalars) {
        this.controlPoints = controlPoints;
        this.speedScalars = speedScalars;
    }

    public LinearTrajectoryBuilder() {
        this(new ArrayList<Pose2D>(), new ArrayList<Pair<Integer, Double>>());
    }

    /**
     * @param pos End of Control Point (Where you want to go essentially)
     * @return A LinearTrajectoryBuilder, with another control point attatched to ControlPoints
     */
    public LinearTrajectoryBuilder moveTo(Pose2D pos) {
        controlPoints.add(pos);
        return new LinearTrajectoryBuilder(controlPoints, speedScalars);
    }

    public LinearTrajectoryBuilder segmentSpeedScalar(int segment, double scalar) {
        speedScalars.add(new Pair<Integer, Double>(segment, scalar));
        return new LinearTrajectoryBuilder(controlPoints, speedScalars);
    }

    public Trajectory build() {

        if (controlPoints.size() <= 1) {
            throw new IllegalArgumentException("Your path size must be greater than 1");
        }

        // samples per segment.
        // make sure to choose a good number for this. primes are pretty bad sample numbers
        // because they cause a lot of round-off error.
        int samples = 100;
        ArrayList<Trajectory.PathSample> pathSamples = new ArrayList<>();

        double accumulatedTime = 0;


        for (int i = 0; i < controlPoints.size() - 1; i++) {
            // create profile, sample 100 times.

            //trapezoidal profile
            Pose2D startPoint = controlPoints.get(i);
            Pose2D endPoint = controlPoints.get(i + 1);

            double startX = startPoint.getX(DistanceUnit.INCH);
            double startY = startPoint.getY(DistanceUnit.INCH);
            double endX = endPoint.getX(DistanceUnit.INCH);
            double endY = endPoint.getY(DistanceUnit.INCH);

            double dx = endX - startX;
            double dy = endY - startY;

            double absDist = Math.sqrt((dx * dx + dy * dy));

            double effMaxV = MAX_V_VERT;
            for (int j = 0; j < speedScalars.size(); j++) {
                if (speedScalars.get(j).getLeft() == i) {
                    effMaxV = MAX_V_VERT * speedScalars.get(j).getRight();
                }
            }

            double accelTime = effMaxV / MAX_A_VERT;
            double accelDist = 0.5 * MAX_A_VERT * Math.pow(accelTime, 2);

            if (accelDist > absDist / 2) {
                accelTime = Math.sqrt(2 * (absDist / 2) / MAX_A_VERT);
                accelDist = absDist / 2;
            }

            double cruiseDist = absDist - 2 * accelDist;
            double cruiseTime = cruiseDist / MAX_V_VERT;

            double totalTime = 2 * accelTime + cruiseTime;

            double heading = Math.atan2(dy, dx); // Radians

            // sampling
            for (int s = 1; s <= samples; s++) {
                Trajectory.PathSample p;
                double time = ((double) s / samples) * totalTime;
                if (time < 0) {
                    p = new Trajectory.PathSample(
                            accumulatedTime,
                            startX,
                            startY,
                            heading,
                            0,
                            0,
                            0
                    );
                } else if (time < accelTime) {
                    double dist = 0.5 * MAX_A_VERT * Math.pow(time, 2);
                    p = new Trajectory.PathSample(
                            time + accumulatedTime,
                            startX + dist * Math.cos(heading),
                            startY + dist * Math.sin(heading),
                            heading,
                            0,
                            0,
                            0
                    );
                } else if (time < accelTime + cruiseTime) {
                    double dist = accelDist + (time - accelTime) * effMaxV;
                    p = new Trajectory.PathSample(
                            time + accumulatedTime,
                            startX + dist * Math.cos(heading),
                            startY + dist * Math.sin(heading),
                            heading,
                            0,
                            0,
                            0
                    );
                } else if (time < 2 * accelTime + cruiseTime) {
                    double t = time - accelTime - cruiseTime;
                    double dist = accelDist + cruiseDist - 0.5 * MAX_A_VERT * Math.pow(t, 2) + MAX_V_VERT * t;
                    p = new Trajectory.PathSample(
                            time + accumulatedTime,
                            startX + dist * Math.cos(heading),
                            startY + dist * Math.sin(heading),
                            heading,
                            0,
                            0,
                            0
                    );
                } else {
                    double dist = 2 * accelDist + cruiseDist;
                    p = new Trajectory.PathSample(
                            totalTime + accumulatedTime,
                            startX + dist * Math.cos(heading),
                            startY + dist * Math.sin(heading),
                            endPoint.getHeading(AngleUnit.DEGREES),
                            0,
                            0,
                            0
                    );
                }
                pathSamples.add(p);
            }
            accumulatedTime += totalTime;
        }
        return new Trajectory(pathSamples);
    }

}
