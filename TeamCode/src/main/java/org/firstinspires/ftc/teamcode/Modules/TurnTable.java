package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.DEGREES_TO_TICKS;
import static org.firstinspires.ftc.teamcode.riptideUtil.TICKS_TO_DEGREES;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KF;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURNTABLE_KP;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class TurnTable {
    static DcMotor motor;

    double goalDeg = 0;
    double prevGoalDeg = 0;
    double goalTicks = 0;

    int startPos;

    PIDController motorController = new PIDController(TURNTABLE_KP, TURNTABLE_KI, TURNTABLE_KD);

    public TurnTable(HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get("turnTable");
        startPos = motor.getCurrentPosition();
    }

    public void setGoalAngle(Double angle) {
        if (angle == null) {
            goalDeg = prevGoalDeg;
        } else {
            prevGoalDeg = goalDeg;
            goalDeg = angle;
        }
        goalTicks = goalDeg * DEGREES_TO_TICKS;
    }

    public int getCurrMotorPos() {
        return motor.getCurrentPosition() - startPos;
    }

    public void zeroMotorPos() {
        startPos = motor.getCurrentPosition();
    }

    /**
     * @param h Robot heading, IN DEGREES
     * @param x dx, IN INCHES
     * @param y dy, IN INCHES
     */
    public void setGoalAngle(double h, double x, double y) {
        double d = (Math.toDegrees(Math.atan2(y, x)) - h);
        if (d < -60) {
            d = -60;
        } else if (d > 60) {
            d = 60;
        }
        setGoalAngle(d);
    }

    public void goToGoalAngle() {

        double localGoalTicks = goalTicks * 3; // gear ratio 1:3

        double currPosTicks = getCurrMotorPos();
        double setPower = motorController.calculate(currPosTicks, localGoalTicks) + TURNTABLE_KF;
        motor.setPower(setPower);
    }

    public void lockOnGoal() {
        goToGoalAngle();
    }

    public double getAngle() {
        return getCurrMotorPos() * TICKS_TO_DEGREES;
    }

    public double getGoalDeg() {
        return goalDeg;
    }

    public void shutOffMotors() {
        motor.setPower(0);
    }

    // tmp func
    public PIDController getPIDController() {
        return motorController;
    }
}
