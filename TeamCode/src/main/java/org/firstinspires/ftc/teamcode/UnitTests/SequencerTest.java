package org.firstinspires.ftc.teamcode.UnitTests;

// idk if this is actually even usefull for reference any more
// up to owen if it should be transfered

import static org.firstinspires.ftc.teamcode.riptideUtil.BOOT_KICKER_RESTING;
import static org.firstinspires.ftc.teamcode.riptideUtil.BOOT_KICKER_UP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.UUID;

@Config
@TeleOp(name = "SequencerTest")
public class SequencerTest extends LinearOpMode {
    Robot robot;
    Outtake out;
    Intake in;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);
        out = new Outtake(hardwareMap);
        in = new Intake(hardwareMap);
        waitForStart();
        if (isStopRequested()) return;

        robot.getOuttake().startFlywheel();

        while (opModeIsActive()) {
            telemetry.addData("Angle ", robot.getIntake().spindexCurrentPosition());
            sequencerTest();
            telemetry.update();
        }
    }
    public boolean hasARun = false;
    public boolean hasBARun = false;
    public boolean hasBBRun = false;
    public int numberXRun = 0;

    public boolean hasALoop = false;
    public boolean hasBALoop = false;
    public boolean hasBBLoop = false;

    public int numberXLoop = 0;
    boolean hasXRun = false;
    int numberBumperRun = 0;

    boolean debounce = false;

    private void sequencerTest() {
        telemetry.addData("actionA",hasARun);
        if (gamepad1.aWasPressed() && robot.s.impulseactions.isEmpty())
            robot.s.addImpulseAction(() -> {
                hasARun = true;
            },1);



        telemetry.addData("actionBA",hasBARun);
        telemetry.addData("actionBB",hasBBRun);

        if (gamepad1.bWasPressed() && robot.s.impulseactions.isEmpty()) {
            robot.s.addImpulseAction(() -> {
                hasBARun = true;
            }, 1);
            robot.s.addImpulseAction(() -> {
                hasBBRun = true;
            },1);
        }
        telemetry.addData("actionXCount",numberXRun);
        if (gamepad1.x) {
            robot.s.addImpulseAction(() -> {
                numberXRun++;
            }, 3);
        }

        if (gamepad1.y) {
            robot.s.addImpulseAction(() -> {
                out.setOuttakeRPM(3000,3000);
            },1d);

            robot.s.addImpulseAction(() -> {
                in.bootkick(BOOT_KICKER_UP);
            },3d);
        }


        telemetry.addData("loopA", hasALoop);
        if (gamepad2.aWasPressed()) {
            robot.s.addLoopAction(() -> {
                hasALoop = true;
                robot.s.getLoopAction("loopA").killAction = true;
            },1,"loopA");
        }

        telemetry.addData("loopBA", hasBALoop);
        telemetry.addData("loopBB", hasBBLoop);

        if (gamepad2.bWasPressed()) {
            robot.s.addLoopAction(() -> {
                hasBALoop = true;
                robot.s.getLoopAction("loopC").killAction = true;
            },1.5,"loopC");
            robot.s.addLoopAction(() -> {
                hasBBLoop = true;
                robot.s.getLoopAction("loopD").killAction = true;
            },2,"loopD");
        }

        telemetry.addData("loopXCount", numberXLoop);

        if (gamepad2.xWasPressed()) {
            hasXRun = true;
            robot.s.addLoopAction(() -> {
                numberXLoop++;
            },1,"loopE");
        }
        if ((!gamepad2.x) && hasXRun) {
            hasXRun = false;
            robot.s.getLoopAction("loopE").killAction = true;
        }

        telemetry.addData("numberBumperRun",numberBumperRun);
        if (gamepad2.leftBumperWasPressed()) {
            robot.s.addRepeatingAction(() -> {
                numberBumperRun++;
            }, 2, UUID.randomUUID().toString());
        }

        if (!(gamepad1.left_bumper || gamepad1.right_bumper)) debounce = false;

        if (!debounce && (gamepad1.left_bumper || gamepad1.right_bumper)) {

            if (gamepad1.left_bumper) {
                robot.s.addImpulseAction(() -> {
                    robot.getOuttake().setOuttakeRPM(2000,2000);
                }, 1);
            }


            if (gamepad1.right_bumper) {
                robot.s.addImpulseAction(() -> {
                    robot.getOuttake().setOuttakeRPM(4000,4000);
                }, 1);
            }

            robot.getIntake().bootkick(BOOT_KICKER_RESTING);

            robot.s.addImpulseAction(() -> {
                robot.getIntake().bootkick(BOOT_KICKER_UP);
            },4);

            robot.s.addImpulseAction(() -> {
                robot.getIntake().bootkick(BOOT_KICKER_RESTING);
            },6);
        }

        robot.s.loop();
        robot.getOuttake().runOuttakePID(telemetry);
        telemetry.update();
    }
}
