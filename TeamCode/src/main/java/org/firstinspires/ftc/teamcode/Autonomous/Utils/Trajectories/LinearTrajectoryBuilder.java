package org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories;

import java.util.ArrayList;

public class LinearTrajectoryBuilder {

    public Trajectory build(){

        return new Trajectory(new ArrayList<Trajectory.PathSample>());
    }

}
