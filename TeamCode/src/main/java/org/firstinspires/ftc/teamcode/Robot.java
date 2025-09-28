package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Slides;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    NormalizedColorSensor colorSensor;

    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "colorSensor");
        drivetrain = new Drivetrain(hardwareMap);
    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

    public NormalizedColorSensor getColorSensor() {
        return colorSensor;
    }
}
