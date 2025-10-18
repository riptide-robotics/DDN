package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.riptideUtil.LONG_DIST_BOT;
import static org.firstinspires.ftc.teamcode.riptideUtil.LONG_DIST_TOP;
import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_BOT;
import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_TOP;
import static org.firstinspires.ftc.teamcode.riptideUtil.SHORT_DIST_BOT;
import static org.firstinspires.ftc.teamcode.riptideUtil.SHORT_DIST_TOP;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Robot;


@Config
@TeleOp(name = "Meet 0 FSM")
public class Meet0FSM extends LinearOpMode {
    //HardwareMap hardwareMap;
    Robot robot;
    AutonomousRobot autoRobot;

    boolean hasrun = false;
    boolean updateTime = false;
    boolean reset = false;

    ElapsedTime endTimer = new ElapsedTime();

    boolean runOuttake = false;

    public static double LONG_DIST_TOP_NOPID = 0.785;
    public static double LONG_DIST_BOT_NOPID = 0.585;

    public static double MID_DIST_TOP_NOPID = 0.52;
    public static double MID_DIST_BOT_NOPID = 0.62;

    public static double SHORT_DIST_TOP_NOPID = 0.48;
    public static double SHORT_DIST_BOT_NOPID = 0.61;

    boolean didRumble = false;


    // DEBOUNCE
    boolean rightBumperPressedG2 = false;
    boolean backPressedG2 = false;
    boolean leftBumperPressedG2 = false;
    boolean leftTriggerPressedG2 = false;
    boolean xPressedG2 = false;

    public enum states{
        TELEOP,
        RESET,
        ENDGAME
    }

    public states currentState = states.TELEOP;

    @Override
    public void runOpMode() throws InterruptedException {

        hasrun = false;

        robot = new Robot(hardwareMap);

        telemetry.addData("Robot status:", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;
        endTimer.reset();
        endTimer.startTime();
        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        while(opModeIsActive()){

            FSM();
            fieldCentricDrive();

            telemetry.addData("Angle: ", robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES));;

            if (updateTime){
                robot.getOuttake().startFlywheel();
            }

            double currTime = endTimer.seconds();

            if (currTime >= 80 && !didRumble){gamepad1.rumble(1, 1, 500); gamepad2.rumble(1, 1, 500);}
            if (currTime >= 85) {didRumble = true;}
            telemetry.addData("Current State:", currentState);
            telemetry.update();
        }
    }


    private void FSM(){
        switch(currentState){
            case TELEOP:
                if (!hasrun){
                    // do teleop setup
                    robot.getIntake().spin(0);
                    robot.getIntake().transfer(0);
                }

                // INTAKE
                if (gamepad2.right_trigger >= 0.1){
                    robot.getIntake().spin(-1);
                    robot.getIntake().transfer(1);
                } else {robot.getIntake().spin(0); robot.getIntake().transfer(0);}

                if (gamepad2.back && !backPressedG2){currentState = states.RESET; backPressedG2 = true;}

//                // TRANSFER
//                if (robot.getOuttake().isAtGoalSpeed() && updateTime) {
//                    robot.getIntake().toggleTransferServo();
//                    robot.getIntake().transferToggle();
//                } else {
//                    robot.getIntake().toggleTransferServo();
//                    robot.getIntake().transferToggle();
//                }



                // OUTTAKE
                if (gamepad2.left_trigger > 0.1 && !runOuttake && !leftTriggerPressedG2){
                    //robot.getOuttake().setFlywheelSpeed(LONG_DIST_TOP, LONG_DIST_BOT /* Long Distance needs to be tuned*/);
                    robot.getOuttake().setFlyWheelPower(LONG_DIST_TOP_NOPID, LONG_DIST_BOT_NOPID);
                    updateTime = true;
                    runOuttake = true;
                    leftTriggerPressedG2 = true;
                } else if (gamepad2.left_bumper && !runOuttake && !leftBumperPressedG2) {
                    //robot.getOuttake().setFlywheelSpeed(MID_DIST_TOP, MID_DIST_BOT /* Short Distance needs to be tuned*/);
                    robot.getOuttake().setFlyWheelPower(MID_DIST_TOP_NOPID, MID_DIST_BOT_NOPID);
                    updateTime = true;
                    runOuttake = true;
                    leftBumperPressedG2 = true;
                } else if (gamepad2.x && !runOuttake && !xPressedG2) {
                    //robot.getOuttake().setFlywheelSpeed(SHORT_DIST_TOP, SHORT_DIST_BOT /* Short Distance needs to be tuned*/);
                    robot.getOuttake().setFlyWheelPower(SHORT_DIST_TOP_NOPID, SHORT_DIST_BOT_NOPID);
                    updateTime = true;
                    runOuttake = true;
                    xPressedG2 = true;
                }

                if (gamepad2.left_trigger > 0.1 && runOuttake && !leftTriggerPressedG2){
                    robot.getOuttake().stop();
                    updateTime = true;
                    runOuttake = false;
                    leftTriggerPressedG2 = true;
                } else if (gamepad2.left_bumper && runOuttake && !leftBumperPressedG2) {
                    robot.getOuttake().stop();
                    updateTime = true;
                    runOuttake = false;
                    leftBumperPressedG2 = true;
                } else if (gamepad2.x && runOuttake && !xPressedG2) {
                    robot.getOuttake().stop();
                    updateTime = true;
                    runOuttake = false;
                    xPressedG2 = true;
                }



                if (gamepad2.dpad_up){
                    currentState = states.ENDGAME;
                    hasrun = false;
                }

                break;

            case RESET:
                reset();
                if (gamepad2.back && !backPressedG2){currentState = states.TELEOP; backPressedG2 = true;}
                if (gamepad2.right_trigger > 0.1){
                    currentState = states.TELEOP; hasrun = false;
                }
                if (gamepad2.left_trigger > 0.1){
                    currentState = states.TELEOP; hasrun = false;
                } else if (gamepad2.left_bumper) {
                    currentState = states.TELEOP; hasrun = false;
                } else if (gamepad2.x) {
                    currentState = states.TELEOP; hasrun = false;
                }
                break;
            case ENDGAME:
                if (!hasrun){
                    robot.getEndgameServos().lift();
                    hasrun = true;
                }

                if (gamepad2.dpad_down){
                    robot.getEndgameServos().lower();
                }

                if (gamepad2.dpad_up){
                    hasrun = false;
                }

                if (gamepad2.b){
                    currentState = states.TELEOP;
                    robot.getEndgameServos().lower();
                    hasrun = false;
                }
        }

        if (!gamepad2.right_bumper){rightBumperPressedG2 = false;}
        if (!gamepad2.left_bumper){leftBumperPressedG2 = false;}
        if (!(gamepad2.left_trigger > 0.1)){leftTriggerPressedG2 = false;}
        if (!gamepad2.x){xPressedG2 = false;}
        if (!gamepad2.back){backPressedG2 = false;}
    }

    private void fieldCentricDrive() {
        double slowdown = gamepad1.right_trigger > 0 ? 0.25 : 1;
        double y = -gamepad1.left_stick_y * slowdown;
        double x = gamepad1.left_stick_x * 1.1 * slowdown;
        double rx = gamepad1.right_stick_x * slowdown;

        double heading = robot.getDrivetrain().getRobotHeading(AngleUnit.RADIANS);


        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frWheelPower = (rotY - rotX - rx) / denominator;
        double flWheelPower = (rotY + rotX + rx) / denominator;
        double brWheelPower = (rotY + rotX - rx) / denominator;
        double blWheelPower = (rotY - rotX + rx) / denominator;

        robot.getDrivetrain().setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);

        if (gamepad1.y) {
            robot.getDrivetrain().resetImu();
        }
    }

    public void reset(){
        robot.getIntake().spin(1);
        robot.getIntake().transfer(-1);
    }



    /*

    -------------------------------------------------------
                               TODO
    --------------------------------------------------------

        1. 3 seperate hard code outputs for outtake // DONE
        2. Reverse intake // DONE
        3. Make servo toggle based on outtake // DONE

     */
    
    
}