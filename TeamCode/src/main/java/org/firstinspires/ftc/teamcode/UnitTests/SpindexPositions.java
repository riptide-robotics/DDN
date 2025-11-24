package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "SpindexPositions")
public class SpindexPositions extends LinearOpMode {

    public static int spindexPosIntake = 1;   // Slot number: 1-3 (-1 = inactive)
    public static int spindexPosOuttake = -1;

    boolean xPressedG2 = false;
    boolean yPressedG2 = false;

    double spindexPos;

    Robot robot;

    public enum UnshiftedPositions {
        SLOT_0_SHOOT(0),
        SLOT_1_SHOOT(120),
        SLOT_2_SHOOT(-120),
        SLOT_0_RECEIVE(180),
        SLOT_1_RECEIVE(-60),
        SLOT_2_RECEIVE(60);

        public final double posUnshifted;
        UnshiftedPositions(double pos) {this.posUnshifted = pos;}
    }

    public enum ShiftedPositions {
        SLOT_0_SHOOT(60),
        SLOT_1_SHOOT(180),
        SLOT_2_SHOOT(-60),
        SLOT_0_RECEIVE(-120),
        SLOT_1_RECEIVE(0),
        SLOT_2_RECEIVE(120);

        public final double posShifted;
        ShiftedPositions(double pos) {this.posShifted = pos;}
    }

    public double getClosestPos(double current, UnshiftedPositions unshifted, ShiftedPositions shifted) {
        double distUnshifted = Math.abs(current - unshifted.posUnshifted);
        double distShifted = Math.abs(current - shifted.posShifted);
        return (distShifted < distUnshifted) ? shifted.posShifted : unshifted.posUnshifted;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {
            cycleSlots();

            if (!gamepad2.x) xPressedG2 = false;
            if (!gamepad2.y) yPressedG2 = false;
        }
    }

    public void cycleSlots() {
        if (gamepad2.y && !yPressedG2) {
            if (spindexPosIntake == -1) spindexPosIntake = 1;
            else if (spindexPosIntake == 1) spindexPosIntake = 2;
            else if (spindexPosIntake == 2) spindexPosIntake = 3;
            else if (spindexPosIntake == 3) spindexPosIntake = 1;

            spindexPosOuttake = -1;
            yPressedG2 = true;
            telemetry.addLine("Current intake slot: " + spindexPosIntake);
        }

        if (gamepad2.x && !xPressedG2) {
            if (spindexPosOuttake == -1) spindexPosOuttake = 1;
            else if (spindexPosOuttake == 1) spindexPosOuttake = 2;
            else if (spindexPosOuttake == 2) spindexPosOuttake = 3;
            else if (spindexPosOuttake == 3) spindexPosOuttake = 1;

            spindexPosIntake = -1;
            xPressedG2 = true;
            telemetry.addLine("Current outtake slot: " + spindexPosOuttake);
        }

        if (spindexPosIntake != -1) {
            if (spindexPosIntake == 1) spindexPos = getClosestPos(robot.getIntake().spindexCurrentPosition(), UnshiftedPositions.SLOT_0_RECEIVE, ShiftedPositions.SLOT_0_RECEIVE);
            else if (spindexPosIntake == 2) spindexPos = getClosestPos(robot.getIntake().spindexCurrentPosition(), UnshiftedPositions.SLOT_1_RECEIVE, ShiftedPositions.SLOT_1_RECEIVE);
            else if (spindexPosIntake == 3) spindexPos = getClosestPos(robot.getIntake().spindexCurrentPosition(), UnshiftedPositions.SLOT_2_RECEIVE, ShiftedPositions.SLOT_2_RECEIVE);
        }
        else if (spindexPosOuttake != -1) {
            if (spindexPosOuttake == 1) spindexPos = getClosestPos(robot.getIntake().spindexCurrentPosition(), UnshiftedPositions.SLOT_0_SHOOT, ShiftedPositions.SLOT_0_SHOOT);
            else if (spindexPosOuttake == 2) spindexPos = getClosestPos(robot.getIntake().spindexCurrentPosition(), UnshiftedPositions.SLOT_1_SHOOT, ShiftedPositions.SLOT_1_SHOOT);
            else if (spindexPosOuttake == 3) spindexPos = getClosestPos(robot.getIntake().spindexCurrentPosition(), UnshiftedPositions.SLOT_2_SHOOT, ShiftedPositions.SLOT_2_SHOOT);
        }

        // Apply to servo
        robot.getIntake().spindexPos2to1Gear(spindexPos);
    }
}
