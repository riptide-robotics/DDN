package org.firstinspires.ftc.teamcode.UnitTests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.PIDController;

@Config
@TeleOp(name = "Turn an angle")
public class TurnAnAngle extends LinearOpMode {
    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

    DcMotor motor;

    public static double kp = 0.002;
    public static double ki = 0.003;
    public static double kd = 0.00005;
    public static double kf = 0;
    public static double deadZone = 5;

    double currPosDeg = 0;
    double currPosTicks = 0;

    public static double goalDeg = 0;
    double prevGoalDeg;
    double goalTicks = 0;
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

        motor = hardwareMap.dcMotor.get("flWheel");

        motor.setDirection(DcMotor.Direction.FORWARD);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorController.setDeadZone(deadZone);

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
            if(prevGoalDeg != (goalDeg * 3)) {
                startTime.reset();
                prevGoalDeg = (goalDeg * 3);
            }
            motorController.setPID(kp, ki, kd);

            goalTicks = (goalDeg * 3) * degreesToTicks;

            currPosTicks = motor.getCurrentPosition();
            motor.setPower(motorController.calculate(currPosTicks, goalTicks) + kf);
            currPosDeg = currPosTicks * ticksToDegrees;

            t.addData("Current Position", currPosDeg / 3);
            t.addData("Goal Position", goalDeg);
            t.update();
        }
    }
}
