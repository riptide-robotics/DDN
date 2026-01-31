package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.riptideUtil.BOOT_KICKER_RESTING;
import static org.firstinspires.ftc.teamcode.riptideUtil.BOOT_KICKER_UP;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.EndgameServos;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Modules.Drivetrain;
import org.firstinspires.ftc.teamcode.Modules.Indicator;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Modules.TurnTable;
import org.firstinspires.ftc.teamcode.Modules.Utils.Sequencer;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

public class Robot {

    public static final VelocityStorage[] shootlookupred = new VelocityStorage[3];
    public static final VelocityStorage[] shootlookupblue = new VelocityStorage[3]; //this will be the lookup tables

    HardwareMap hardwareMap;

    Indicator indicator;

    Drivetrain drivetrain;

    Intake intake;

    EndgameServos endgameServos;

    Outtake outtake;

    Camera camera;

    TurnTable turntable;

    riptideUtil.TEAM_COLOR alliance = riptideUtil.TEAM_COLOR.RED; // change based on alliance

    /**TODO we're going to have to loop this*/
    public Sequencer s;

    char[] motifOrder;
    final char[] badChar = {'b', 'b', 'b'};

    public enum StartPos {
        CLOSE_START,
        FAR_START,
        TELEOP_RESET,
        NOT_SET,
    }

    private StartPos startStatus = StartPos.NOT_SET;

    public Robot(HardwareMap map) {
        hardwareMap = map;

        indicator = new Indicator(hardwareMap, true);
        drivetrain = new Drivetrain(hardwareMap);
        s = new Sequencer(drivetrain);
        intake = new Intake(hardwareMap);
        endgameServos = new EndgameServos(hardwareMap);
        outtake = new Outtake(hardwareMap);
        camera = new Camera(hardwareMap, alliance);
        turntable = new TurnTable(hardwareMap);

        motifOrder = camera.scanMotifOrder();
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Intake getIntake() {
        return intake;
    }

    public EndgameServos getEndgameServos() {
        return endgameServos;
    }

    public Outtake getOuttake() {
        return outtake;
    }

    public Camera getCamera() {
        return camera;
    }

    public TurnTable getTurntable() {
        return turntable;
    }

    /**
     * TODO: This is going to run into a NPE if the camera doesn't see the goal
     */
    public void setFlyWheelPowerOnDistance(boolean run, Telemetry tele) {
        double distance = camera.getGoalDistance();
        outtake.setPowerOnDist(distance, run, tele);
    }

    @Placeholder(note = "atomic amount of framework here")
    public void shootbasedoncolor(char color) {
        if (true) /*shut up IDE*/ throw new UnsupportedOperationException("Can't do this yet!");
        rotateToColor(color);
        if (true) /*TODO, run if no color*/ {
            //vibrate(); todo
            return;
        }
        char[] artifacts = new char[3]; //todo get chars from the thing
        byte b = 0; //todo get current position of spindexer
        if (artifacts[b] == color) {/*can eject NOW*/}
        ;
        //rotate to required position
        double[] velocity = getOuttakeRPMS(true /*todo again get team color here*/);
        outtake.setOuttakeRPM(velocity[1], velocity[2]);
    }

    @Placeholder
    public void /*figure out what we are returning later*/ rotateToColor(char color) {
        throw new UnsupportedOperationException("Can't rotate just yet");
    }

    public void alignTurretToGoal() {
        switch (startStatus) {
            case NOT_SET:
                // Indicator Flash
                turntable.setGoalAngle(0.0);
            case CLOSE_START: {
                double dx = riptideUtil.CLOSE_START_GOAL_POS.getX(DistanceUnit.INCH) - getDrivetrain().getCurrPos().getX(DistanceUnit.INCH);
                double dy = riptideUtil.CLOSE_START_GOAL_POS.getY(DistanceUnit.INCH) - getDrivetrain().getCurrPos().getY(DistanceUnit.INCH);
                double h = getDrivetrain().getCurrPos().getHeading(AngleUnit.DEGREES);
                turntable.setGoalAngle(h, dx, dy);
            }
            case FAR_START: {
                double dx = riptideUtil.FAR_START_GOAL_POS.getX(DistanceUnit.INCH) - getDrivetrain().getCurrPos().getX(DistanceUnit.INCH);
                double dy = riptideUtil.FAR_START_GOAL_POS.getY(DistanceUnit.INCH) - getDrivetrain().getCurrPos().getY(DistanceUnit.INCH);
                double h = getDrivetrain().getCurrPos().getHeading(AngleUnit.DEGREES);
                turntable.setGoalAngle(h, dx, dy);
            }
        }
    }

    public StartPos getStartStatus() {
        return startStatus;
    }

    public void setStartStatus(StartPos ss) {
        startStatus = ss;
    }

    @Placeholder
    public void alignTurretToRedGoal() {
        if (1 + 1 == 2) throw new UnsupportedOperationException("Can't align just yet");

        double error = camera.getAngleToRedGoal();
        turntable.setGoalAngle(/*get current angle of turret*/ 0 - error);


    }

    @Placeholder
    public void alignTurretToBlueGoal() {
        if (1 + 1 == 2) throw new UnsupportedOperationException("Can't align just yet");

        double error = camera.getAngleToBlueGoal();
        turntable.setGoalAngle(/*get current angle of turret*/ 0 - error);

    }


    /**
     * Set indicator light based upon current spindexer. Put this in a loop.
     *
     * @param colorRequested The color that is checked for existence. Use a space for no check.
     **/
    public void setStatus(char colorRequested) {
        char[] order = new char[]{intake.slot0CurrColor(), intake.slot1CurrColor(), intake.slot2CurrColor()};

        boolean contains =
                order[0] == colorRequested ||
                        order[1] == colorRequested ||
                        order[2] == colorRequested;

        if (colorRequested == ' ') contains = true;


        byte amount = (byte) (
                (order[0] != ' ' ? 1 : 0) +
                        (order[1] != ' ' ? 1 : 0) +
                        (order[2] != ' ' ? 1 : 0));


        if (amount == 0) indicator.setRGB(Indicator.IndColor.ORANGE);
        else if (amount == 3 && !contains) indicator.setRGB(Indicator.IndColor.RED);

        else if (!contains && colorRequested == 'p') indicator.setRGB(Indicator.IndColor.BLUE);
        else if (!contains && colorRequested == 'g') indicator.setRGB(Indicator.IndColor.YELLOW);
        else if (contains && colorRequested == 'p') indicator.setRGB(Indicator.IndColor.INDIGO);
        else if (contains && colorRequested == 'g') indicator.setRGB(Indicator.IndColor.GREEN);

        else indicator.setRGB(Indicator.IndColor.NONE); //err

    }

    /**
     * Set the indicator status of the robot, with respect to the number of artifacts in the trough.
     */
    public void setStatus(byte artifactsInTrough) {
        if (artifactsInTrough < 0) artifactsInTrough = 0; //warn the driver??

        if (Arrays.equals(motifOrder, badChar))
            motifOrder = camera.scanMotifOrder(); //is there a problem
        if (Arrays.equals(motifOrder, badChar)) { //is there still a problem
            indicator.setRGB(Indicator.IndColor.NONE);
            return;
        }

        setStatus(motifOrder[artifactsInTrough % 3]);
    }

    /**
     * Designed and intended for testing, but might work before an actual match.
     */
    public void setTeamColor(riptideUtil.TEAM_COLOR color) {
        alliance = color;
    }

    /**
     * get flywheel RPMs based upon distance. Not tested.
     */
    public double[] getOuttakeRPMS(boolean onBlueGoal) {
        double distance = onBlueGoal ? camera.getDistanceToBlueGoal() : camera.getDistanceToRedGoal();
        double[] returned = new double[2];

        VelocityStorage[] velocities = onBlueGoal ? shootlookupblue : shootlookupred;


        for (int i = 0; i < velocities.length; i++) {
            VelocityStorage curr = velocities[i];

            if ((i + 1 >= velocities.length && distance >= curr.distance) ||
                    (distance >= curr.distance && distance <= curr.distance)) {

                returned[0] = curr.lowerRPM;
                returned[1] = curr.upperRPM;
            }
        }
        return returned;
    }


    /**
     * Only access outside of Robot for the purposes of reading data from it.
     */
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

    public void outtake(double slotNum, Telemetry tele) {
        s.addImpulseAction(() -> {
            intake.bootkick(BOOT_KICKER_UP);
            tele.addLine("Boot kicker up");
        }, 1);
        s.addImpulseAction(() -> {
            intake.bootkick(BOOT_KICKER_RESTING);
            tele.addLine("Boot kicker down");
        },1);
        s.addImpulseAction(() -> {
            if (slotNum == 0) {
                Intake.SLOT_0 = 'b';
            }
            if (slotNum == 1) {
                Intake.SLOT_1 = 'b';
            }
            if (slotNum == 2) {
                Intake.SLOT_2 = 'b';
            }
            riptideUtil.nextShotAvailable = true;
            tele.addLine("setting lost to blank");
        },1);
    }
}
