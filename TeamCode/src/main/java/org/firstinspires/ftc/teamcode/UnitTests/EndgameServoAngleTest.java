package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.EndgameServos;

@Config
@TeleOp(name="Endgame Angle Test", group="Unit Test")
public class EndgameServoAngleTest extends LinearOpMode {
    EndgameServos endgameServos;
    public static double lift;
    public static double zero;

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */

        endgameServos = new EndgameServos(hardwareMap);

        waitForStart();
        if (isStopRequested()) return;

        // * * * * * * * * * * * * * * *
        // * Start button clicked
        // * * * * * * * * * * * * * * *

        telemetry.clear();
        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */

        while(opModeIsActive()) {
            if (gamepad1.x) {
                endgameServos.testLift(lift);
            }
            if (gamepad1.y) {
                endgameServos.testLower(zero);
            }
        }
    }
}
