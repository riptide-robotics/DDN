package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.DummyClasses.EndgameServos;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    Intake intake;

    EndgameServos endgameServos;

    Outtake outtake;

    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);

        intake = new Intake(hardwareMap);

        endgameServos = new EndgameServos(hardwareMap);

        outtake = new Outtake(hardwareMap);
    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

    public Intake getIntake() {return intake;}

    public EndgameServos getEndgameServos() {return endgameServos;}

    public Outtake getOuttake() {return outtake;}
}
