package org.firstinspires.ftc.teamcode.UnitTests;

import static org.firstinspires.ftc.teamcode.riptideUtil.nextShotAvailable;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Modules.Intake;

import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Sequence Shooting")
public class SequenceShooting extends LinearOpMode {
    Robot robot;
    boolean xPressedG2 = false;
    double spindexPosOuttake = 0;
    @Override
    public void runOpMode() throws InterruptedException{
        robot = new Robot(hardwareMap);
        robot.getIntake().initSpindex();
        Intake.SLOT_0 = 'g';
        Intake.SLOT_1 = 'g';
        Intake.SLOT_2 = 'g';
        spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
        waitForStart();
        while(opModeIsActive()){
            if (gamepad2.x && !xPressedG2) {
                telemetry.addLine("X pressed G2");
                spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
                robot.outtake(spindexPosOuttake, telemetry);
                xPressedG2 = true;
            }

            if (gamepad2.x){
                telemetry.addLine("xpressed");
            }
            if (gamepad2.b){
                Intake.SLOT_0 = 'g';
                Intake.SLOT_1 = 'g';
                Intake.SLOT_2 = 'g';
            }

            if (!gamepad2.x){
                xPressedG2 = false;
            }

            telemetry.addData("Outtake Slot: ", spindexPosOuttake);
            telemetry.addData("Slot 0: ", Intake.SLOT_0);
            telemetry.addData("Slot 1: ", Intake.SLOT_1);
            telemetry.addData("Slot 2: ", Intake.SLOT_2);
            telemetry.update();
        }
    }

}
