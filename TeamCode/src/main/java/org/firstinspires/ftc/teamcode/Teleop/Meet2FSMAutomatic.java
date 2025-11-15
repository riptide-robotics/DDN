package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_RESTING;
import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_UP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.Modules.Camera;
import org.firstinspires.ftc.teamcode.Robot;


@Config
@TeleOp(name = "Meet 2 FSM Automatic")
public class Meet2FSMAutomatic extends LinearOpMode {
    //HardwareMap hardwareMap;
    Robot robot;
    //AutonomousRobot autoRobot;

    boolean hasrun = false;
    boolean updateTime = false;
    boolean reset = false;
    boolean align = false;

    public static double intakeSpeed = 0.6;

    boolean runSpindex = false;

    ElapsedTime endTimer = new ElapsedTime();

    boolean runOuttake = false;

    public static double currentTopRPMGoal;
    public static double currentBottomRPMGoal;

    boolean didRumble = false;

    boolean canBootKick = false;


    // DEBOUNCE
    boolean rightBumperPressedG2 = false;
    boolean backPressedG2 = false;
    boolean leftBumperPressedG2 = false;
    boolean leftTriggerPressedG2 = false;
    boolean xPressedG2 = false;
    boolean rightPressedG2 = false;
    boolean rightTriggerPressedG2 = false;

    Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    public enum states{
        TELEOP,
        ENDGAME
    }

    public states currentState = states.TELEOP;

    @Override
    public void runOpMode() throws InterruptedException {

        hasrun = false;

        robot = new Robot(hardwareMap);

        telemetry.addData("Robot status:", "succesfully initiated");
        telemetry.update();

        Camera.processors_enabled processor = Camera.processors_enabled.ALL;
        robot.getCamera().setPipeline(processor);

        waitForStart();
        if (isStopRequested()) return;
        endTimer.reset();
        endTimer.startTime();
        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        robot.getOuttake().startFlywheel();

        while(opModeIsActive()){

            FSM();
            // fieldCentricDrive();
            tankDrive();
            robot.getCamera().runCamera(processor);
            robot.setFlyWheelPowerOnDistance(runOuttake, tele);
            robot.getIntake().CanBootKick(canBootKick);

            double currTime = endTimer.seconds();

            if (currTime >= 80 && !didRumble){gamepad1.rumble(1, 1, 500); gamepad2.rumble(1, 1, 500);}
            if (currTime >= 82) {didRumble = true;}


            tele.update();
        }
    }


    private void FSM(){
        switch(currentState){
            case TELEOP:
                if (!hasrun){
                    // do teleop setup
                    robot.getIntake().spin(0);
                }

//                if (gamepad1.x && !xPressedG2){/* align */ align = true; xPressedG2 = true;}
//                 else if (gamepad1.x && !xPressedG2){align = false; xPressedG2 = true;}

                // INTAKE
                if (gamepad2.right_trigger >= 0.1){
                    robot.getIntake().spin(intakeSpeed);
                    robot.getIntake().movetoEmptySlot();
                    runSpindex = true;
                } else if (gamepad2.back){robot.getIntake().spin(-intakeSpeed);}
                else {robot.getIntake().spin(0);}


                // Boot Kicker
                if (gamepad2.dpad_up && robot.getOuttake().isAtGoalSpeed() && canBootKick){/* robot.getIntake().spinSpindex(-1); */ robot.getIntake().BootKick(SPINDEX_ARM_UP);}
                else {/* robot.getIntake().spinSpindex(1); */ robot.getIntake().ResetBootKick(SPINDEX_ARM_RESTING);}

//                if (gamepad2.dpad_right && !rightPressedG2 && !runOuttake){
//                    currentTopRPMGoal = -1;
//                    currentBottomRPMGoal = -1;
//                    updateTime = true;
//                    runOuttake = true;
//                    rightPressedG2 = true;
//                }

                // OUTTAKE
                if (gamepad2.left_trigger > 0.1 && !runOuttake && !leftTriggerPressedG2){
                    runOuttake = true;
                    leftTriggerPressedG2 = true;
                }

                if (gamepad2.left_trigger > 0.1 && runOuttake && !leftTriggerPressedG2){
                    runOuttake = false;
                    leftTriggerPressedG2 = true;
                } // else if (gamepad2.dpad_right && runOuttake && !rightPressedG2) {currentTopRPMGoal = 0;
////                    currentBottomRPMGoal = 0;
//                    updateTime = false;
//                    runOuttake = false;
//                    rightPressedG2 = true;}

                // ENDGAME
                if (gamepad2.y){
                    currentState = states.ENDGAME;
                    hasrun = false;
                }

                break;
            case ENDGAME:
                if (!hasrun){
                    robot.getEndgameServos().lift();
                    hasrun = true;
                }

                if (gamepad2.a){
                    robot.getEndgameServos().lower();
                }

                if (gamepad2.y){
                    hasrun = false;
                }

                if (gamepad2.b){
                    currentState = states.TELEOP;
                    robot.getEndgameServos().lower();
                    hasrun = false;
                }


        }
        if (!gamepad2.dpad_right){rightPressedG2 = false;}
        if (!gamepad2.right_bumper){rightBumperPressedG2 = false;}
        if (!gamepad2.left_bumper){leftBumperPressedG2 = false;}
        if (!(gamepad2.left_trigger > 0.1)){leftTriggerPressedG2 = false;}
        if (!(gamepad2.right_trigger > 0.1)) rightTriggerPressedG2 = false;
        if (!gamepad1.x){xPressedG2 = false;}
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

    public void tankDrive() {
        double slowdown = gamepad1.right_trigger > 0 ? 0.25 : 1;
        robot.getDrivetrain().setWheelPowers(
                -gamepad1.left_stick_y * slowdown,
                -gamepad1.right_stick_y * slowdown,
                -gamepad1.right_stick_y * slowdown,
                -gamepad1.left_stick_y * slowdown
        );
    }
}