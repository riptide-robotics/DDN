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

    public static boolean dpadUpPressed = false;
    public static boolean dpadDownPressed = false;

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
            double pos = endgameServos.getPos();
            if (gamepad1.dpad_up && !dpadUpPressed) {
                endgameServos.testLift(pos + 0.1);
                dpadUpPressed = true;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }
            if (gamepad1.dpad_down && !dpadDownPressed) {
                endgameServos.testLift(pos - 0.1);
                dpadDownPressed = true;
            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }
            if (gamepad1.x) {
                endgameServos.lower();
            }
            telemetry.addData("currPos", endgameServos.getPos());
            telemetry.update();
        }
    }
}
