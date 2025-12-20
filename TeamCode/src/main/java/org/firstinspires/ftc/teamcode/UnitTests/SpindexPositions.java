package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "SpindexPositions")
public class SpindexPositions extends LinearOpMode {

    public static double spindexPosIntake = -1;
    public static double spindexPosOuttake = -1;

    boolean xPressedG2 = false;
    boolean yPressedG2 = false;
    boolean bPressedG2 = false;
    boolean aPressedG2 = false;
    boolean dUpPressedG2 = false;
    boolean dRightPressedG2 = false;


    double spindexPos;

    Robot robot;

    public static slotStatus SLOT_0 = slotStatus.BLANK;
    public static slotStatus SLOT_1 = slotStatus.BLANK;
    public static slotStatus SLOT_2 = slotStatus.BLANK;

    public static double pos;

    public enum slotStatus{
        BLANK, GREEN, PURPLE
    }
//
//    public UnshiftedPositions currentState = UnshiftedPositions.SLOT_0_SHOOT;
    public  int currAngle;
//
//    public enum UnshiftedPositions {
//        SLOT_0_SHOOT(180),
//        SLOT_1_SHOOT(60),
//        SLOT_2_SHOOT(-60),
//        SLOT_0_RECEIVE(0),
//        SLOT_1_RECEIVE(120),
//        SLOT_2_RECEIVE(-120);
//
//
//        public final int posUnshifted;
//        UnshiftedPositions(int pos) {this.posUnshifted = pos;}
//    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        currAngle = 450;

        robot.getIntake().setSpindexPosition(currAngle);
        waitForStart();

        while (opModeIsActive()) {
            cycleSlots();
//
//            if (!gamepad2.x) xPressedG2 = false;
//            if (!gamepad2.y) yPressedG2 = false;
        }
    }

    public void cycleSlots() {
        if (gamepad2.y && !yPressedG2) {
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_RECEIVE, telemetry);
            yPressedG2 = true;
        }

        if (gamepad2.x && !xPressedG2) {
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_1_RECEIVE, telemetry);
            xPressedG2 = true;
        }

        if (gamepad2.a && !aPressedG2){
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_2_RECEIVE, telemetry);
            aPressedG2 = true;
        }

        if (gamepad2.b && !bPressedG2){
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_SHOOT, telemetry);
            bPressedG2 = true;
        }

        if (gamepad2.dpad_up && !dUpPressedG2){
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_1_SHOOT, telemetry);
            dUpPressedG2 = true;
        }

        if (gamepad2.dpad_right && !dRightPressedG2){
            robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_2_SHOOT, telemetry);
            dRightPressedG2 = true;
        }



        if (!gamepad2.x){xPressedG2 = false;}
        if (!gamepad2.y){yPressedG2 = false;}
        if (!gamepad2.a){aPressedG2 = false;}
        if (!gamepad2.dpad_right){
            dRightPressedG2 = false;}
        if (!gamepad2.dpad_up){dUpPressedG2 = false;}
        if(!gamepad2.b){bPressedG2 = false;}

//        robot.getIntake().spin(-1);

        telemetry.update();
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
