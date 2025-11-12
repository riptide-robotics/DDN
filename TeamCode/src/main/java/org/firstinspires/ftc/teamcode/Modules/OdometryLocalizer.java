package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;

/**
 * For three-wheel code, refer back to the Meet2 repo for the Into the Deep Season
 */
public class OdometryLocalizer implements Runnable {

    private boolean running = true;
    private final double pollRate;
    private double currTime = 0;
    private double previousTime = 0;

    GoBildaPinpointDriver pinpoint;

    public OdometryLocalizer(GoBildaPinpointDriver pinpoint, int pollRate){
        this.pollRate = pollRate;
        this.pinpoint = pinpoint;
    }

    public EditablePose2D getCurrPos(){
        return new EditablePose2D(
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                pinpoint.getHeading(AngleUnit.RADIANS),
                DistanceUnit.INCH
        );
    }

    public void start(){
        pinpoint.resetPosAndIMU();
        currTime = System.nanoTime()/1e6;
    }

    public void stop(){
        running = false;
    }

    @Override
    public void run(){
        while (running) {
            currTime = System.nanoTime() / 1e6;
            if (currTime - previousTime >= pollRate) {
                pinpoint.update();
                previousTime = currTime;
            }
            try {
                Thread.sleep((long) (pollRate / 2)); // sleep half poll period
            } catch (InterruptedException ignored) {}
        }
    }
}
