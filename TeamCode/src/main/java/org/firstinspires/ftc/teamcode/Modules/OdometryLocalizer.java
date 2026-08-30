package org.firstinspires.ftc.teamcode.Modules;

// need to add support for 3 wheel code just in case

// NEEDS REFRACTORING WHEN TRANSFER
// READY TO TRANSFER

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.riptideUtil;

/**
 * For three-wheel code, refer back to the Meet2 repo for the Into the Deep Season
 */
public class OdometryLocalizer implements Runnable {

    private boolean running = true;
    private final double pollRate;
    private double currTime = 0;
    private double previousTime = 0;

    GoBildaPinpointDriver pinpoint;

    public OdometryLocalizer(GoBildaPinpointDriver pinpoint, int pollRate) {
        this.pollRate = pollRate;
        this.pinpoint = pinpoint;
    }

    public Pose2D getCurrPos() {
        return new Pose2D(
                DistanceUnit.INCH,
                pinpoint.getPosX(DistanceUnit.INCH),
                pinpoint.getPosY(DistanceUnit.INCH),
                AngleUnit.RADIANS,
                pinpoint.getHeading(AngleUnit.RADIANS)
        );
    }

    public void start() {
        pinpoint.resetPosAndIMU();
        pinpoint.setPosition(riptideUtil.START_POSITION);
        currTime = System.nanoTime() / 1e6;
    }

    public void stop() {
        running = false;
    }

    @Override
    public void run() {
        while (running) {
            currTime = System.nanoTime() / 1e6;
            if (currTime - previousTime >= pollRate) {
                pinpoint.update();
                previousTime = currTime;
            }
            try {
                Thread.sleep((long) (pollRate / 2)); // sleep half poll period
            } catch (InterruptedException ignored) {
            }
        }
    }
}
