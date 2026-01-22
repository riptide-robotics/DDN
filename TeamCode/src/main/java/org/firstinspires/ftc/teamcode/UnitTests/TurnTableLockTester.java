package org.firstinspires.ftc.teamcode.UnitTests;

import static org.firstinspires.ftc.teamcode.riptideUtil.START_POSITION;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.TurnTable;
import org.firstinspires.ftc.teamcode.Robot;

@Config
@TeleOp(name = "Turntable Lock Tester")
public class TurnTableLockTester extends LinearOpMode {
    Robot robot;
    DcMotor frWheel, flWheel, brWheel, blWheel;
    TurnTable turnTable;
    boolean lockOn = false;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap);


        frWheel = hardwareMap.dcMotor.get("frWheel");
        flWheel = hardwareMap.dcMotor.get("flWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");

        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        frWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        brWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        brWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        blWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        robot.getDrivetrain().getPinpoint().setPosition(START_POSITION);
        robot.getDrivetrain().getPinpoint().update();

        turnTable = new TurnTable(hardwareMap);

        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        /*
         * * * * * * * * * * * * * * *
         * Start button clicked
         * * * * * * * * * * * * * * *
         */
        robot.getDrivetrain().getPinpoint().recalibrateIMU();
        robot.getDrivetrain().getPinpoint().setPosition(START_POSITION);
        telemetry.clear();

        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */

        while(opModeIsActive()) {
            tankDrive();


            Pose2D currPos = robot.getDrivetrain().getPinpoint().getPosition();

            telemetry.addLine(currPos.toString() + '\n');

            if(gamepad1.a){
                robot.getDrivetrain().getPinpoint().setPosition(START_POSITION);
            }
            if(gamepad1.x) {
                robot.getDrivetrain().getPinpoint().recalibrateIMU();
                telemetry.addLine("IMU recalibrated");
            }
            if (gamepad1.y && !lockOn) {
                turnTable.setGoalAngle(
                        robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES),
                        robot.getDrivetrain().getCurrPos().getX(DistanceUnit.INCH),
                        robot.getDrivetrain().getCurrPos().getY(DistanceUnit.INCH)
                );
                telemetry.addLine("locking on goal: " + turnTable.getGoalDeg());
                // I need the position of the goal ToT
                lockOn = true;
            } else if (gamepad1.y && lockOn) {
                lockOn = false;
            }
            if (lockOn) {
                turnTable.lockOnGoal();
            }
            telemetry.addData("turnTable angle", turnTable.getAngle());

            telemetry.addData("X Position", currPos.getX(DistanceUnit.INCH));
            telemetry.addData("Y Position", currPos.getY(DistanceUnit.INCH));
            telemetry.addData("Orientation (Degrees)", Math.toDegrees(currPos.getHeading(AngleUnit.RADIANS)));

            telemetry.addLine("\n IMU measured heading \n")
                    .addData("Orientation (Degrees)", robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES));

            telemetry.update();
            robot.getDrivetrain().getPinpoint().update();
        }
    }

    public void tankDrive(){
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;

        robot.getDrivetrain().setWheelPowers(leftPower, rightPower, rightPower, leftPower);
    }
}
