package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Slides;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    /**
     * Creates a new Robot object. Please note that this
     * does not create an entire new digital reference to
     * Robot, rather it simply creates a way to access it.
     */
    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);
    }

    /**
     * This is a getter for the robot's DriveTrain. This can
     * be used to, for example, set voltages to the wheels.
     */
    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

}
