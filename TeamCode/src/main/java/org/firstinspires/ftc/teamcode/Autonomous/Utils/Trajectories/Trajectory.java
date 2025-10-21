package org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories;

import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

public abstract class Trajectory {
    public abstract EditablePose2D getPosition(double dt);
}
