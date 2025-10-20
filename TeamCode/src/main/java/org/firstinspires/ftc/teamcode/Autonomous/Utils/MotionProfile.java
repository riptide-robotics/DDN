package org.firstinspires.ftc.teamcode.Autonomous.Utils;

import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

public abstract class MotionProfile {
    public abstract EditablePose2D getPosition(double dt);
}
