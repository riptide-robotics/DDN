package org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Path;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;

import java.sql.Array;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

public class HeadingBehavior{

    private final Trajectory trajectory;

    public enum InterpolatingMode {
        HEAD_TANGENT, // runs tangent to path,
        HEAD_FLOAT
    }

    private final ArrayList<InterpolatingMode> modes = new ArrayList<>();

    public HeadingBehavior(Trajectory trajectory){
        this.trajectory = trajectory;
        for(int i = 0; i < trajectory.getSplineLength(); i++){
            modes.add(null);
        }
    }

    public HeadingBehavior setModeToHeadTangent(int index){
        modes.set(index, InterpolatingMode.HEAD_TANGENT);
        return this;
    }

    public HeadingBehavior setAllSegmentsToHeadTangent(){
        for(int i = 0; i< trajectory.getSplineLength(); i++){
            modes.set(i, InterpolatingMode.HEAD_TANGENT);
        }
        return this;
    }

    public HeadingBehavior setModeToHeadFloat(int index) {
        modes.set(index, InterpolatingMode.HEAD_FLOAT);
        return this;
    }

    public HeadingBehavior setAllSegmentsToHeadFloat(){
        for(int i = 0; i < trajectory.getSplineLength(); i++){
            modes.set(i, InterpolatingMode.HEAD_FLOAT);
        }
        return this;
    }

    public static boolean allNonNull(List<?> list) {
        return list.stream().allMatch(Objects::nonNull);
    }

    public double getHeading(double percentOf){
        if (modes.isEmpty()){ throw new IllegalStateException("You need to populate the heading behavior for each trajectory segment");}
        if (allNonNull(modes)) { throw new IllegalStateException("The amount of set heading behaviors needs to be the same as your spline segment amount");}



        double elapsedTime = 0;
        for (int i = 0; i < trajectory.getSplineLength(); i++){
            if(elapsedTime + )
        }

    }



}
