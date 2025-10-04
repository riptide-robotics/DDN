package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Slides;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    Intake intake;

    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);

        intake = new Intake(hardwareMap);
    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

    public Intake getIntake() {return intake;}

}
