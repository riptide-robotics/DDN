package org.firstinspires.ftc.teamcode.Teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Autonomous.AutonomousRobot;
import org.firstinspires.ftc.teamcode.DummyClasses.Outtake;
import org.firstinspires.ftc.teamcode.DummyClasses.TwoServos;
import org.firstinspires.ftc.teamcode.Robot;


@Config
@TeleOp(name = "Meet 0 FSM")
public class Meet0FSM extends LinearOpMode {
    //HardwareMap hardwareMap;

    Outtake outtake;
    TwoServos twoServos;

    Robot robot;
    AutonomousRobot autoRobot;

    boolean hasrun = false;

    ElapsedTime endTimer = new ElapsedTime();


    public enum states{
        IDLE,
        INTAKE,
        OUTTAKE,
        ENDGAME
    }

    public states currentState = states.IDLE;

    @Override
    public void runOpMode() throws InterruptedException {

        hasrun = false;


        outtake = new Outtake(hardwareMap);
        twoServos = new TwoServos(hardwareMap);

        telemetry.addData("Robot status:", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        telemetry.clear();
        telemetry.addData("Robot status", "Started!");
        telemetry.update();

        while(opModeIsActive()){
            outtake.controlPitch(gamepad2.left_stick_y);
            fieldCentricDrive();

            double currTime = endTimer.seconds();

            if (currTime >= 80){gamepad1.rumble(1, 1, 500); gamepad2.rumble(1, 1, 500);}

            telemetry.addData("Current State:", currentState);
            telemetry.update();
        }
    }


    private void FSM(){
        switch(currentState){
            case IDLE:
                if (!hasrun){
                    //Do idle things
                    hasrun = true;
                }

                if (gamepad2.y){
                    currentState = states.INTAKE;
                    hasrun = false;
                }

                if (gamepad2.x){
                    currentState = states.OUTTAKE;
                    hasrun = false;
                }

                if (gamepad2.dpad_up){
                    currentState = states.ENDGAME;
                    hasrun = false;
                }

                break;
            case INTAKE:
                if (!hasrun){
                    //Do intake setup
                    hasrun = true;
                }

                if (gamepad2.x){
                    currentState = states.OUTTAKE;
                    hasrun = false;
                }

                if (gamepad2.b){
                    currentState = states.IDLE;
                    hasrun = false;
                }

                if (gamepad2.dpad_up){
                    currentState = states.ENDGAME;
                    hasrun = false;
                }

                break;
            case OUTTAKE:
                if (!hasrun){
                    //Do outtake setup
                    hasrun = true;
                }

                if (gamepad2.x){
                    outtake.start(1);
                }

                if (gamepad2.b){
                    currentState = states.IDLE;
                    hasrun = false;
                }

                if (gamepad2.left_trigger > 0.1){
                    currentState = states.INTAKE;
                    hasrun = false;
                }

                if (gamepad2.dpad_up){
                    currentState = states.ENDGAME;
                    hasrun = false;
                }


                break;
            case ENDGAME:
                if (!hasrun){
                    twoServos.lift();
                    hasrun = true;
                }

                if (gamepad2.dpad_down){
                    twoServos.lower();
                }

                if (gamepad2.dpad_up){
                    twoServos.lift();
                }

                if (gamepad2.b){
                    currentState = states.IDLE;
                    twoServos.lower();
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
