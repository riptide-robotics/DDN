package org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories;

import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

/**
 * An Abstract class to give some global methods to any new Spline or Path that other people want to implement in the future
 * Just note that whatever you add to this list, every Trajectory made will need to have these methods
 * choose wisely
 */
public abstract class Trajectory {
    public abstract EditablePose2D getPosition(double dt);

    public abstract int getSplineLength();
}
