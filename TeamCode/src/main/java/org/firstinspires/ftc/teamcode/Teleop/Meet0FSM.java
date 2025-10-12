package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.teamcode.riptideUtil.LONG_DIST_BOT;
import static org.firstinspires.ftc.teamcode.riptideUtil.LONG_DIST_TOP;
import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_BOT;
import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DiST_TOP;
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

    ElapsedTime endTimer = new ElapsedTime();

    boolean runFlywheel = false;

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

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        while(opModeIsActive()){

            FSM();
            fieldCentricDrive();

            if (updateTime){
                robot.getOuttake().startFlywheel();
            }

            double currTime = endTimer.seconds();

            if (currTime >= 80){gamepad1.rumble(1, 1, 500); gamepad2.rumble(1, 1, 500);}

            telemetry.addData("Current State:", currentState);
            telemetry.update();
        }
    }


    private void FSM(){
        switch(currentState){
            case TELEOP:
                if (!hasrun){
                    // do teleop setup

                }


                if (gamepad2.right_trigger > 0.1){
                    robot.getIntake().spin(1);
                } else {
                    robot.getIntake().spin(0);
                }

                if (gamepad2.left_trigger > 0.1){
                    robot.getOuttake().setFlywheelSpeed(LONG_DIST_TOP, LONG_DIST_BOT /* Long Distance needs to be tuned*/);
                    updateTime = true;
                } else if (gamepad2.left_bumper) {
                    robot.getOuttake().setFlywheelSpeed(MID_DiST_TOP, MID_DIST_BOT /* Short Distance needs to be tuned*/);
                    updateTime = true;
                } else if (gamepad2.x) {
                    robot.getOuttake().setFlywheelSpeed(SHORT_DIST_TOP, SHORT_DIST_BOT /* Short Distance needs to be tuned*/);
                    updateTime = true;
                }else{
                    robot.getOuttake().stop();
                    updateTime = false;
                }

                if (gamepad2.right_bumper) {
                    robot.getIntake().transferToggle();
                }

                if (gamepad2.dpad_up){
                    currentState = states.ENDGAME;
                    hasrun = false;
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
}