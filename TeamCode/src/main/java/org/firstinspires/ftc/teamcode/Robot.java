package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_RESTING;
import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_UP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.DummyClasses.EndgameServos;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Indicator;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.Utils.Sequencer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class Robot {

    public static final VelocityStorage[] shootlookupred = new VelocityStorage[5];
    public static final VelocityStorage[] shootlookupblue = new VelocityStorage[5]; //this will be the lookup tables

    HardwareMap hardwareMap;

    Indicator indicator;

    Drivetrain drivetrain;

    Intake intake;

    EndgameServos endgameServos;

    Outtake outtake;

    Camera camera;

    riptideUtil.TEAM_COLOR alliance = riptideUtil.TEAM_COLOR.RED; // change based on alliance

    public Sequencer s;

    static {

        shootlookupblue[0] = new VelocityStorage();
        shootlookupblue[1] = new VelocityStorage();
        shootlookupblue[2] = new VelocityStorage();
        shootlookupblue[3] = new VelocityStorage();
        shootlookupblue[4] = new VelocityStorage();

        shootlookupred[0] = new VelocityStorage();
        shootlookupred[1] = new VelocityStorage();
        shootlookupred[2] = new VelocityStorage();
        shootlookupred[3] = new VelocityStorage();
        shootlookupred[4] = new VelocityStorage();

    }

    public Robot (VelocityStorage[] shootlookupblue, HardwareMap map){
        hardwareMap = map;

        indicator = new Indicator(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);
        s = new Sequencer(drivetrain);
        intake = new Intake(hardwareMap);
        endgameServos = new EndgameServos(hardwareMap);
        outtake = new Outtake(hardwareMap);
        camera = new Camera(hardwareMap, alliance);
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

    @Placeholder(note = "atomic amount of framework here")
    public void shootbasedoncolor(char color) {
        if (true) /*shut up IDE*/ throw new UnsupportedOperationException("Can't do this yet!");
        rotateToColor(color);
        if (true) /*TODO, run if no color*/ {


        }
        else {
            //TODO get through lookup table
            outtake.setOuttakeRPM(3000,3000);
            s.addImpulseAction(() -> {
                intake.bootkick(SPINDEX_ARM_UP);
            }, 2);
            s.addImpulseAction(() -> {
                intake.bootkick(SPINDEX_ARM_RESTING);
            },4);
        }
    }
    @Placeholder
    public void /*figure out what we are returning later*/ rotateToColor(char color) {
        throw new UnsupportedOperationException("Can't rotate just yet");
    }

    @Placeholder
    public void alignTurretToRedGoal() {
        if (1+1==2) throw new UnsupportedOperationException("Can't align just yet");

        double error = camera.getAngleToRedGoal();
        outtake.SetTurretGoalAngle(/*get current angle of turret*/ 0 - error);


    }
    @Placeholder
    public void alignTurretToBlueGoal() {
        if (1+1==2) throw new UnsupportedOperationException("Can't align just yet");

        double error = camera.getAngleToBlueGoal();
        outtake.SetTurretGoalAngle(/*get current angle of turret*/ 0 - error);

    }
    @Placeholder
    public void aimToRedGoal(){ // copy for blue
        double dist = camera.getDistanceToRedGoal();
        /**
         * split maximum shooting distance into 5 "sections", set flywheel powers based on a lookup table (array)
         * based on which section we fall into.
         */
    }
    @Placeholder
    public void aimToBlueGoal(){ // copy for blue
        double dist = camera.getDistanceToBlueGoal();
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

        if (amount == 0) indicator.setStatusColor(Indicator.statusLights.EMPTY);

        if ((amount == 1 || amount == 2) && contains) indicator.setStatusColor(Indicator.statusLights.SEMI_OPEN);
        if ((amount == 1 || amount == 2) && !contains) indicator.setStatusColor(Indicator.statusLights.SEMI_OPEN_AND_NONE_REQUESTED);

        if (amount == 3 && contains) indicator.setStatusColor(Indicator.statusLights.FULL_SPINDEXER);
        if (amount == 3 && !contains) indicator.setStatusColor(Indicator.statusLights.FULL_SPINDEXER_AND_NONE_REQUESTED);
    }

    public void setTeamColor(riptideUtil.TEAM_COLOR color) {
        alliance = color;
    }

    /**Only access outside of Robot for the purposes of reading data from it.*/
    private static class VelocityStorage {
        public double upperRPM;
        public double lowerRPM;
        public double distance;

        public VelocityStorage(double upperRPM, double lowerRPM, double distanceININCHES) {
            this.upperRPM = upperRPM;
            this.lowerRPM = lowerRPM;
            this.distance = distanceININCHES;
        }
        public VelocityStorage() {
            this.upperRPM = 0;
            this.lowerRPM = 0;
            this.distance = 0;
        }


    }
}
