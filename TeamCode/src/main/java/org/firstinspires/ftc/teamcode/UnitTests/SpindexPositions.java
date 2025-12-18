package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "SpindexPositions")
public class SpindexPositions extends LinearOpMode {

    public static double spindexPosIntake = -1;
    public static double spindexPosOuttake = -1;

    boolean xPressedG2 = false;
    boolean yPressedG2 = false;

    double spindexPos;

    Robot robot;

    public static slotStatus SLOT_0 = slotStatus.BLANK;
    public static slotStatus SLOT_1 = slotStatus.BLANK;
    public static slotStatus SLOT_2 = slotStatus.BLANK;

    public static double pos;

    public enum slotStatus{
        BLANK, GREEN, PURPLE
    }

    public UnshiftedPositions currentState = UnshiftedPositions.SLOT_0_SHOOT;
    public  int currAngle;

    public enum UnshiftedPositions {
        SLOT_0_SHOOT(180),
        SLOT_1_SHOOT(60),
        SLOT_2_SHOOT(-60),
        SLOT_0_RECEIVE(0),
        SLOT_1_RECEIVE(120),
        SLOT_2_RECEIVE(-120);


        public final int posUnshifted;
        UnshiftedPositions(int pos) {this.posUnshifted = pos;}
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        currAngle = 450;
        robot.getIntake().spindexPos2to1Gear(currAngle);
        currentState = UnshiftedPositions.SLOT_0_RECEIVE;
        waitForStart();

        while (opModeIsActive()) {
            cycleSlots();
//
//            if (!gamepad2.x) xPressedG2 = false;
//            if (!gamepad2.y) yPressedG2 = false;
        }
    }

    public void cycleSlots() {
        if (gamepad2.y) {
            goTo(UnshiftedPositions.SLOT_0_RECEIVE);
        }

        if (gamepad2.x) {
            goTo(UnshiftedPositions.SLOT_1_RECEIVE);
        }

        if (gamepad2.a){
            goTo(UnshiftedPositions.SLOT_2_RECEIVE);
        }

        if (gamepad2.b){
            goTo(UnshiftedPositions.SLOT_0_SHOOT);
        }

        if (gamepad2.dpad_up){
            goTo(UnshiftedPositions.SLOT_1_SHOOT);
        }

        if (gamepad2.dpad_right){
            goTo(UnshiftedPositions.SLOT_2_SHOOT);
        }

        telemetry.update();
    }
    public int diff;
    public void goTo(UnshiftedPositions goal) {
        diff = goal.posUnshifted - currentState.posUnshifted;
        if (diff < -180) {
            diff += 360;
        }
        if (diff > 180) {
            diff -= 360;
        }
        telemetry.addData("Diff ", diff);

        int newAngle = currAngle + diff;
        if (newAngle < 0) {newAngle += 360;}
        if (newAngle > 890) {newAngle -= 360;}
        currAngle = newAngle;
        robot.getIntake().spindexPos2to1Gear(newAngle);
        telemetry.addData("New angle ", newAngle);
        telemetry.addData("Servo Angle ", robot.getIntake().spindexCurrentPosition());
        currentState = goal;
    }

    public double pos(UnshiftedPositions goal) {
        double diff = goal.posUnshifted - currentState.posUnshifted;
        if (diff < -180) {
            diff += 360;
        }
        if (diff > 180) {
            diff -= 360;
        }

        double newAngle = robot.getIntake().spindexCurrentPosition() + diff;
        if (newAngle < 0) {newAngle += 360;}
        if (newAngle > 890) {newAngle -= 360;}
        return newAngle;
    }

    public double getNextIntakeSlot() {
        if (SLOT_0 == slotStatus.BLANK) return 0;
        if (SLOT_1 == slotStatus.BLANK) return 1;
        if (SLOT_2 == slotStatus.BLANK) return 2;
        return -1;
    }

    public int getNextOuttakeSlot() {
        if (SLOT_0 != slotStatus.BLANK) return 0;
        if (SLOT_1 != slotStatus.BLANK) return 1;
        if (SLOT_2 != slotStatus.BLANK) return 2;
        return -1;
    }
}
