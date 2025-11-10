package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Outtake;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    Outtake outtake;

    Camera camera;

    public Robot (HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;

        drivetrain = new Drivetrain(hardwareMap);

        outtake = new Outtake(hardwareMap);

        camera = new Camera(hardwareMap);
    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

    public Outtake getOuttake() {return outtake;}

    public Camera getCamera() {return camera;}

    public void setFlyWheelPowerOnDistance(boolean run, Telemetry tele){
        Double distance = camera.getGoalDistance();
        if (distance != null) {
            outtake.setPowerOnDist(distance, run, tele);
        }
    }
}