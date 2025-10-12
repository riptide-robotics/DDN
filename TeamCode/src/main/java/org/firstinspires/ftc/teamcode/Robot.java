package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.DummyClasses.EndgameServos;
import org.firstinspires.ftc.teamcode.DummyClasses.IntakeMeet0;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Slides;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    IntakeMeet0 intake;

    EndgameServos endgameServos;

    Outtake outtake;

    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);

        intake = new IntakeMeet0(hardwareMap);

        endgameServos = new EndgameServos(hardwareMap);

        outtake = new Outtake(hardwareMap);
    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

    public IntakeMeet0 getIntake() {return intake;}

    public EndgameServos getEndgameServos() {return endgameServos;}

    public Outtake outtake() {return outtake;}
}
