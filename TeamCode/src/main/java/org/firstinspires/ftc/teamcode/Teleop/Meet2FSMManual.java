package org.firstinspires.ftc.teamcode.Teleop;

    import static org.firstinspires.ftc.teamcode.riptideUtil.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Intake;
import org.firstinspires.ftc.teamcode.Robot;


@Config
@TeleOp(name = "Meet 2 FSM Manual")
public class Meet2FSMManual extends LinearOpMode {
    Robot robot;


    boolean hasrun = false;
    boolean recieve = false;
    boolean outtake = false;
    public static double spin = 1;

    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public enum states {
        TELEOP,
        ENDGAME
    }

    public states currentState = states.TELEOP;

    /**
     * SPINDEX
     **/
    public static double spindexPosIntake = 1;
    public static double spindexPosOuttake = -1;

    /**
     * OUTTAKE
     **/
    public static double currentTopRPMGoal;
    public static double currentBottomRPMGoal;


    /**
     * TIMERS
     **/
    ElapsedTime spindexDelayTimer;
    ElapsedTime scanDelayTimer;
    public static double spindexDelay = 750; // IN MS
    public static double scanDelay = 1000;
    boolean moveToNextOuttakeSlot = false;
    int ballsintrough = 0;
    boolean checkGhostScan = false;

        /**            DEBOUNCE             **/
        boolean rightBumperPressedG2 = false;
        boolean backPressedG2 = false;
        boolean leftBumperPressedG2 = false;
        boolean leftTriggerPressedG2 = false;
        boolean xPressedG2 = false;
        boolean rightPressedG2 = false;
        boolean yPressedG2 = false;
        boolean bPressedG2 = false;
        boolean aPressedG2 = false;
        boolean dUpPressedG2 = false;
        boolean dDownPressedG2 = false;



        @Override
        public void runOpMode() throws InterruptedException {
            hasrun = false;
            robot = new Robot(hardwareMap);
            spindexDelayTimer = new ElapsedTime();
            scanDelayTimer = new ElapsedTime();
            recieve = true;
            outtake = false;
            moveToNextSlot = false;
            moveToNextOuttakeSlot = false;
            nextShotAvailable = true;
            checkGhostScan = false;
            spindexPosIntake = robot.getIntake().getNextIntakeSlot();

        robot.getIntake().initSpindex();
        robot.getIntake().initColorSensor();

        telemetry.addData("Robot status:", "succesfully initiated");
        telemetry.update();

        currentTopRPMGoal = 0;
        currentBottomRPMGoal = 0;

        ElapsedTime e = new ElapsedTime();
        e.reset();
        boolean d = false;

        while (!d && e.milliseconds() < 5000) {
            telemetry.addData("Time Left to Choose", 2000 - e.milliseconds());
            if (gamepad1.dpad_up) {
                d = true;
                robot.setStartStatus(Robot.StartPos.FAR_START);
            } else if (gamepad1.dpad_down) {
                d = true;
                robot.setStartStatus(Robot.StartPos.CLOSE_START);
            } else if (gamepad1.x) {
                d = true;
                robot.setStartStatus(Robot.StartPos.NOT_SET);
            }
            telemetry.update();
        }

        if (!d) {
            telemetry.addData("YOU HAVE RAN OUT OF TIME. GO TO THE RESET POSITION AND PRESS", true);
            telemetry.addData("GAMEPAD 1 DPAD DOWN", true);
            robot.setStartStatus(Robot.StartPos.NOT_SET);
        } else {
            telemetry.addData("YOU HAVE CHOSEN", robot.getStartStatus());
        }
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        robot.getOuttake().startFlywheel();
        robot.getDrivetrain().startOdometry();

        while (opModeIsActive()) {

            FSM();
            tankDrive();
            robot.getOuttake().setOuttakeRPM(currentTopRPMGoal, currentBottomRPMGoal);
            robot.getOuttake().runOuttakePID(tele);
            robot.s.loop();
            robot.setStatus((byte) Intake.ballsShot);
            robot.getTurntable().goToGoalAngle();

//                double currTime = endTimer.seconds();
//                robot.getOuttake().mapJoyToAngle(gamepad2.right_stick_x);
//                robot.getOuttake().updateTurntableAngle(tele);
            tele.update();
        }
    }


    private void FSM() {
        processTroughCounter();

        switch (currentState) {
            case TELEOP:
                if (!hasrun) {
                    currentTopRPMGoal = 0;
                    currentBottomRPMGoal = 0;
                    hasrun = true;
                }

                if (gamepad1.dpad_down){
                   robot.getDrivetrain().getPinpoint().setPosition(START_POSITION);
                   robot.setStartStatus(Robot.StartPos.CLOSE_START);
                }

                idleSpin();
                cycleSlots();
                count();

                if (gamepad1.dpad_up){
                    currentState = states.ENDGAME;
                }
                // ENDGAME
                //                if (gamepad2.dpad_up){
                //                    currentState = states.ENDGAME;
                //                    hasrun = false;
                //                }

                break;
            case ENDGAME:
                if (!hasrun) {
                    robot.getEndgameServos().lift(endgameServosUp);
                    hasrun = true;
                }

                if (gamepad1.dpad_down) {
                    robot.getEndgameServos().lower();
                }

                if (gamepad1.dpad_up) {
                    hasrun = false;
                }

                if (gamepad1.b) {
                    currentState = states.TELEOP;
                    robot.getEndgameServos().lower();
                    hasrun = false;
                }

                break;
        }
        if (!gamepad2.dpad_right) {
            rightPressedG2 = false;
        }
        if (!gamepad2.right_bumper) {
            rightBumperPressedG2 = false;
        }
        if (!gamepad2.left_bumper) {
            leftBumperPressedG2 = false;
        }
        if (!(gamepad2.left_trigger > 0.1)) {
            leftTriggerPressedG2 = false;
        }
        if (!gamepad2.x) {
            xPressedG2 = false;
        }
        if (!gamepad2.y) {
            yPressedG2 = false;
        }
        if (!gamepad2.back) {
            backPressedG2 = false;
        }
        if (!gamepad2.dpad_up) {
            dUpPressedG2 = false;
        }
        if (!gamepad2.dpad_down) {
            dDownPressedG2 = false;
        }
        if (!gamepad2.a) {
            aPressedG2 = false;
        }
    }

    public void tankDrive() {
        double slowdown = gamepad1.right_trigger > 0 ? 0.25 : 1;
        robot.getDrivetrain().setWheelPowers(
                -gamepad1.left_stick_y * slowdown,
                -gamepad1.right_stick_y * slowdown,
                -gamepad1.right_stick_y * slowdown,
                -gamepad1.left_stick_y * slowdown
        );
    }


    public void count() {
        if (gamepad2.dpad_up && !dUpPressedG2) {
            robot.getIntake().increaseCount();
            dUpPressedG2 = true;
        }

        if (gamepad2.dpad_down && !dDownPressedG2) {
            robot.getIntake().decreseCount();
            dDownPressedG2 = true;
        }

            if (gamepad2.back){
                robot.getIntake().resetCount();
            }
            tele.addData("Current Count: ", robot.getIntake().getCount());
        }

    public void cycleSlots() {
        if (gamepad2.y && !yPressedG2) {
            scanDelayTimer.reset();
            scanDelayTimer.startTime();

                checkGhostScan = false;
//                recieve = true;
            currentTopRPMGoal = 0;
            currentBottomRPMGoal = 0;
            outtake = false;
//                recieve = true;
            spindexPosIntake = robot.getIntake().getNextIntakeSlot();
            robot.getIntake().moveToNextIntakeSlot(spindexPosIntake);
            yPressedG2 = true;

            robot.getTurntable().setGoalAngle(0.0);
        }

        if (!checkGhostScan && scanDelayTimer.milliseconds() >= scanDelay && !outtake) {
            recieve = true;
            checkGhostScan = true;
        }

            if (recieve){
                robot.getIntake().setSlotColor(spindexPosIntake);
                if (robot.getIntake().checkColor() != 'b' && !delayIntake){
                    delayIntake = true;
                    robot.s.addImpulseAction(() -> {
                        delayIntake = false;
                    }, 0.5);
                }
                spindexPosOuttake = -1;
                if (moveToNextSlot) {
                    spindexPosIntake = robot.getIntake().getNextIntakeSlot();
                    robot.getIntake().moveToNextIntakeSlot(spindexPosIntake);
                    robot.getIntake().delayMovementToNextSlot(spindexDelayTimer, spindexDelay, tele);
                }

            tele.addLine("Spindex is Receiving");
            tele.addData("moveToNextSlot: ", moveToNextSlot);
        }

            if (gamepad2.x && !xPressedG2) {
                outtake = true;
                recieve = false;
                currentTopRPMGoal = MID_DIST_TOP;
                currentBottomRPMGoal = MID_DIST_BOT;
                spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
                xPressedG2 = true;
            }

            if (gamepad2.right_trigger > 0.1){
                currentTopRPMGoal = 0;
                currentBottomRPMGoal = 0;
            }

            if (gamepad2.b) {
                outtake = true;
                recieve = false;
                currentTopRPMGoal = SHORT_DIST_TOP;
                currentBottomRPMGoal = SHORT_DIST_BOT;
                spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
            }

            if (gamepad2.left_trigger > 0.1){
                outtake = true;
                recieve = false;
                currentTopRPMGoal = LONG_DIST_TOP;
                currentBottomRPMGoal = LONG_DIST_BOT;
                spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
            }

            if (outtake){
                spindexPosIntake = -1;
                robot.getIntake().cycleOuttakeSlot(spindexPosOuttake);
                if (gamepad2.a && !aPressedG2 && nextShotAvailable) {
                    nextShotAvailable = false;
                    robot.outtake(spindexPosOuttake, tele);
                    aPressedG2 = true;
                }

            if (nextShotAvailable) {
                spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
            }

                /*************************************************
                                Add turntable stuff here
                 *************************************************/


            if (gamepad2.left_trigger > 0) {
                robot.alignTurretToGoal();
            } else {
                robot.getTurntable().setGoalAngle(0.0);
            }

        }
        tele.addData("Intake Slot: ", spindexPosIntake);
        tele.addData("Outtake Slot: ", spindexPosOuttake);
        tele.addData("Slot 0: ", Intake.SLOT_0);
        tele.addData("Slot 1: ", Intake.SLOT_1);
        tele.addData("Slot 2: ", Intake.SLOT_2);
        tele.addData("Current Color: ", robot.getIntake().checkColor());
        tele.addData("Scan delay: ", scanDelayTimer.milliseconds());
        tele.addData("Robot X", robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH));
        tele.addData("Robot Y", robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH));
//            tele.addData("Is at goal speed ", robot.getOuttake().isAtGoalSpeed());
//            tele.addData("Next Slot Available ", nextShotAvailable);
        }

        public void idleSpin(){
            if (gamepad2.right_trigger > 0.1) {
                robot.getIntake().spin(0);
            } else {
                robot.getIntake().spin(spin);
            }
        }

    public void processTroughCounter() {
        if (gamepad1.leftBumperWasPressed()) ballsintrough--;
        if (gamepad1.rightBumperWasPressed()) ballsintrough++;
        if (ballsintrough < 0) ballsintrough = 0;
    }
}
