package org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.Pair;

import java.util.ArrayList;


// This is for tank drive, for holonomic drive, Turning while moving is possible
// if you need help expanding to holonomic drive trains, you can always hit me up
// --Aaron Xie
public class LinearTrajectoryBuilder {
    ArrayList<Pose2D> controlPoints;
    ArrayList<Pair<Integer, Double>> speedScalars;

    public LinearTrajectoryBuilder(ArrayList<Pose2D> controlPoints, ArrayList<Pair<Integer, Double>> speedScalars) {
        this.controlPoints = controlPoints;
        this.speedScalars = speedScalars;
    }

    public LinearTrajectoryBuilder() {
        this(new ArrayList<Pose2D>(), new ArrayList<Pair<Integer, Double>>());
    }

    /**
     * First turns towards angle needed, then moves towards the specified point.
     *
     * @param pos End of Control Point (Where you want to go essentially)
     * @return A LinearTrajectoryBuilder, with another control point attatched to ControlPoints
     */
    public LinearTrajectoryBuilder moveTo(Pose2D pos) {
        controlPoints.add(pos);
        return new LinearTrajectoryBuilder(controlPoints, speedScalars);
    }

    public LinearTrajectoryBuilder segmentSpeedScalar(Pair<Integer, Double> s) {
        speedScalars.add(s);
        return new LinearTrajectoryBuilder(controlPoints, speedScalars);
    }

    public Trajectory build() {
        //first turn, then move

        double totalTime = 0;


        return new Trajectory(new ArrayList<Trajectory.PathSample>());
    }

}
