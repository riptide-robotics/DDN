package org.firstinspires.ftc.teamcode.Tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Better Spindex Positions", group = "Tuning")
public class SpindexTuning extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        robot.getIntake().initSpindex();

        waitForStart();

        while (opModeIsActive()) {

            setSpindexPositions();

        }

    }

    void setSpindexPositions() {
        //copied from spindex positions. Added zero spindex option
        if (gamepad2.y) {
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_RECEIVE);
        }

        if (gamepad2.x) {
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_1_RECEIVE);
        }

        if (gamepad2.a) {
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_2_RECEIVE);
        }

        if (gamepad2.b) {
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_SHOOT);
        }

        if (gamepad2.dpad_up) {
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_1_SHOOT);
        }

        if (gamepad2.dpad_right) {
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_2_SHOOT);
        }

        if (gamepad2.dpad_down) {
            robot.getIntake().zeroSpindex();
        }


    }
}

