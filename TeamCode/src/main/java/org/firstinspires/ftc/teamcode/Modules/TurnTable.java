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

    PIDController motorController = new PIDController(TURNTABLE_KP, TURNTABLE_KI, TURNTABLE_KD);

    public TurnTable(HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get("turnTable");
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
    public void goToGoalAngle() {

        double goalTicks = goalDeg * DEGREES_TO_TICKS;

        double currPosTicks = motor.getCurrentPosition();
        double setPower = motorController.calculate(currPosTicks, goalTicks) + TURNTABLE_KF;
        motor.setPower(setPower);
    }
    public void lockOnGoal(double h, double x, double y) {
        double d = Math.atan2(y, x) - h;
        if (d < -90) {
            d = -90;
        } else if (d > 90) {
            d = 90;
        }
        setGoalAngle(d);
        goToGoalAngle();
    }
    public double getAngle() {
        return motor.getCurrentPosition() * TICKS_TO_DEGREES;
    }
}
