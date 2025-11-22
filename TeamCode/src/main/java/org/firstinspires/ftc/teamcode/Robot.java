package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DummyClasses.EndgameServos;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Sequencer;

public class Robot {

    HardwareMap hardwareMap;

    Drivetrain drivetrain;

    Intake intake;

    EndgameServos endgameServos;

    Outtake outtake;

    Camera camera;

    Sequencer s;

    public Robot (HardwareMap map){
        hardwareMap = map;

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        endgameServos = new EndgameServos(hardwareMap);
        outtake = new Outtake(hardwareMap);
        camera = new Camera(hardwareMap);
    }

    public Drivetrain getDrivetrain(){
        return drivetrain;
    }

    public Intake getIntake() {return intake;}

    public EndgameServos getEndgameServos() {return endgameServos;}

    public Outtake getOuttake() {return outtake;}

    public Camera getCamera() {return camera;}

    public void setFlyWheelPowerOnDistance(boolean run, Telemetry tele){
        double distance = camera.getGoalDistance();
        outtake.setPowerOnDist(distance, run, tele);
    }

    public void shootbasedoncolor() {
        if (true) /*shut up IDE*/ throw new UnsupportedOperationException("Can't do this yet!");
        rotateToColor();
        if (true) /*TODO, run if no color*/ {
            //TODO too: vibrate to warn driver

            return;
        }
        else {
            s.addAction(() -> {
                intake.BootKick(0 /*TODO no clue what goes here*/);
                //TODO remove color from position
            }, 0.5);
        }
    }
    public void /*figure out what we are returning later*/ rotateToColor() {
        throw new UnsupportedOperationException("Can't rotate just yet");
    }
    public void alignTurretToRedGoal() {
        double error = 0; //getAngleToRedGoal(); TODO set this up somewhere?
        //outtake.setGoalAngle(outtake.getCurrentAngle() + error) TODO too
        throw new UnsupportedOperationException("Can't align just yet");
    }

    public void alignTurretToBlueGoal() {
        double error = 0; //getAngleToBlueGoal(); TODO set this up somewhere?
        //outtake.setGoalAngle(outtake.getCurrentAngle() + error) TODO too
        throw new UnsupportedOperationException("Can't align just yet");
    }
    public void aimToRedGoal(){ // copy for blue
        double dist = 0; //camera.getdisttoredgoal(); yeah TODO too
        /**
         * split maximum shooting distance into 5 "sections", set flywheel powers based on a lookup table (array)
         * based on which section we fall into.
         */
    }

    public void aimToBlueGoal(){ // copy for blue
        double dist = 0; //camera.getdisttobluegoal(); yeah TODO too
        /**
         * split maximum shooting distance into 5 "sections", set flywheel powers based on a lookup table (array)
         * based on which section we fall into.
         */
    }
    public void setStatus() {
        //set currently nonexistent status lights
        throw new UnsupportedOperationException("Light not set up");
    }


}
