package org.firstinspires.ftc.teamcode.Modules;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class VelocityPidController {
    private double kI = 0;
    private double kP = 0;
    private double kD = 0;
    private double integral = 0;
    private double lastError = 0;

    public void setPID(double kP, double kI, double kD){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.lastError = 0;
    }

    public double calculate(double targetVelocity, double currentVelocity, double dt){
        double error = targetVelocity - currentVelocity;

        integral += error * dt;

        double derivative = (error - lastError) / dt;

        double output = (kP*error) + (kI * integral) + (kD * derivative);

        lastError = error;

        return output;
    }

    public void reset(){
        integral = 0;
        lastError = 0;
    }

}
