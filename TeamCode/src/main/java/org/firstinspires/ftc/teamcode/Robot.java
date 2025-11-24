package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DummyClasses.EndgameServos;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Indicator;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Sequencer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Field;
import java.util.ArrayList;

import javax.annotation.processing.AbstractProcessor;

public class Robot {

    private static final Logger log = LoggerFactory.getLogger(Robot.class);
    HardwareMap hardwareMap;

    Indicator indicator;

    Drivetrain drivetrain;

    Intake intake;

    EndgameServos endgameServos;

    Outtake outtake;

    Camera camera;

    public Sequencer s;

    public Robot (HardwareMap map){
        hardwareMap = map;

        s = new Sequencer();
        indicator = new Indicator(hardwareMap);
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

    /**TODO: This is going to run into a NPE if the camera doesn't see the goal*/
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
    /**
     * Set indicator light based upon current spindexer. Put this in a loop.
     * <br><br>
     * I hate this thing.
     * */
    public void setStatus(char color) throws NoSuchFieldException, IllegalAccessException {
        // i -really- don't want to touch intake with someone else works on it. This will do until it can be set to public or something
        //ABSOLUTELY DO NOT TOUCH THIS until it can be replaced
        Field infield = Intake.class.getDeclaredField("order");
        infield.setAccessible(true);
        final char[] order = (char[]) infield.get(intake);
        infield.setAccessible(false);

        boolean contains =
                order[0] == color ||
                order[1] == color ||
                order[2] == color ;

        byte amount = (byte) (
                (order[0] != ' ' ? 1:0) +
                (order[1] != ' ' ? 1:0) +
                (order[2] != ' ' ? 1:0) );

        if (amount == 0) indicator.setIndicatorLights(Indicator.statusLights.EMPTY);

        if ((amount == 1 || amount == 2) && contains) indicator.setIndicatorLights(Indicator.statusLights.SEMI_OPEN);
        if ((amount == 1 || amount == 2) && !contains) indicator.setIndicatorLights(Indicator.statusLights.SEMI_OPEN_AND_NONE_REQUESTED);

        if (amount == 3 && contains) indicator.setIndicatorLights(Indicator.statusLights.FULL_SPINDEXER);
        if (amount == 3 && !contains) indicator.setIndicatorLights(Indicator.statusLights.FULL_SPINDEXER_AND_NONE_REQUESTED);
    }


}
