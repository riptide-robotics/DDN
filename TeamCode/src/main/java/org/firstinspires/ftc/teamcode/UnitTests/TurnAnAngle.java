package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.PIDController;

@Config
public class TurnAnAngle extends LinearOpMode {

    DcMotor motor;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;

    double currPosDeg = 0;
    double currPosTicks = 0;

    double goalDeg = 0;
    double prevGoalDeg;
    double goalTicks = 0;
    double prevGoalTicks;
    static final double ticksToDegrees = 360/751.8;
    static final double degreesToTicks = 751.8/360;

    public ElapsedTime startTime= new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * * * * * * * * * * * * * * *
         * INITIALIZATION
         * * * * * * * * * * * * * * *
         */
        PIDController motorController = new PIDController(kp, ki, kd);

        motor = hardwareMap.dcMotor.get("motor");

        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addData("Robot status", "succesfully initiated");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // * * * * * * * * * * * * * * *
        // * Start button clicked
        // * * * * * * * * * * * * * * *

        telemetry.clear();
        /*
         * * * * * * * * * * * * * * *
         * LOOP
         * * * * * * * * * * * * * * *
         */

        while(opModeIsActive()) {
            if(prevGoalDeg != goalDeg) {
                startTime.reset();
                prevGoalDeg = goalDeg;
            }
            motorController.setPID(kp, ki, kd);

            goalTicks = goalDeg * degreesToTicks;

            motor.setPower(motorController.calculate(currPosTicks, goalTicks) + kf);

//            if(gamepad1.y){
//                motor.setPower(1);
//            }else{
//                motor.setPower(0);
//            }

            telemetry.addData("Current Position", currPosDeg);
            telemetry.addData("Goal Position", goalDeg);
            telemetry.update();
        }
    }
}
