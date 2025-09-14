package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Slides;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    public GoBildaPinpointDriver odoComputer;

    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);



        odoComputer = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

    public GoBildaPinpointDriver getOdoComputer(){
        return odoComputer;
    }
}
