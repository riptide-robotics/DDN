package org.firstinspires.ftc.teamcode.Tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DummyClasses.Outtake;
import org.firstinspires.ftc.teamcode.Modules.VelocityPidController;

@Config
@TeleOp(name = "Velocity Pid Tuner", group = "Tuning")
public class VelocityPidTuner extends LinearOpMode {

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double targetvelocity = 500;
    double prevTarget;

    public static double ticksPerRev = 28; // IDK what motors we have

    Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


    private DcMotor motor;

    Outtake outtake;
    VelocityPidController PIDController;

    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws  InterruptedException{

        outtake = new Outtake(hardwareMap);


        PIDController  = new VelocityPidController();
        PIDController.setPID(kP, kI, kD);

        waitForStart();

        double lastPos = outtake.currPos();
        double lastTime = timer.seconds();

        while(opModeIsActive()){

            if (prevTarget != targetvelocity){
                prevTarget = targetvelocity;
                PIDController.reset();
                timer.reset();
            }


            double currentTime = timer.seconds();
            double currentPos = outtake.currPos();
            double currentVelocity = getMotorVelocity(lastPos, lastTime, ticksPerRev, currentTime);

            double dt = currentTime - lastTime;
            double output = PIDController.calculate(targetvelocity, currentVelocity, dt);

            output = Math.max(-1.0, Math.min(1.0, output));

            outtake.start(output);



            t.addData("Current Velocity ", currentVelocity);
            t.addData("Goal Velocity ", targetvelocity);
            t.addData("Output ", output);
            t.update();

            lastPos = currentPos;
            lastTime = currentTime;
        }
    }

    public double getMotorVelocity(double lastPos, double lastTime, double ticksPerRev, double currentTime){
        double currentPos = outtake.currPos();
        double dt = currentTime - lastTime;

        double ticksPerSec = (currentPos - lastPos) / dt;

        return ticksPerSec / ticksPerRev;
    }
}
