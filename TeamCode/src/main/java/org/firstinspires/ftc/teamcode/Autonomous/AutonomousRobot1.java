package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.LAT_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.POINT_TOLERANCE;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.VERT_KP;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.MotionProfile;
import org.firstinspires.ftc.teamcode.Modules.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.riptideUtil;

public class AutonomousRobot1 extends Robot {

    private EditablePose2D startPose = new EditablePose2D(0, 0, Math.toRadians(90), DistanceUnit.INCH);
    private MotionProfile profile;

    public AutonomousRobot1(HardwareMap hardwareMap) {
        super(hardwareMap);
    }
    /**
     * @param curr, robot current heading in degrees
     * @param goal, robot goal heading, global, in degrees
     * @return an angle within 180 and -180
     */
    private double normalizeAngleError(double curr, double goal) {
        double error = goal - curr;
        error = ((error + 180) % 360 + 360) % 360 - 180;
        return error;
    }

    /**
     * @param robotPos Pose2D of the robot
     * @param point    Pose2D of our goal point (note that we don't need the heading).
     * @return True if the robot is within the determined boundaries of a point
     */
    public boolean atPoint(EditablePose2D robotPos, EditablePose2D point) {

        double dx = point.getX(DistanceUnit.INCH) - robotPos.getX(DistanceUnit.INCH);
        double dy = point.getY(DistanceUnit.INCH) - robotPos.getY(DistanceUnit.INCH);

        double distance = Math.sqrt(dx * dx + dy * dy);
        return distance <= POINT_TOLERANCE;
    }

    public enum moveStates {
        GOTOPOINT,
        STAYATPOINT,
        CALCULATE,
        IDLE,
        ONLYACTION
    }

    public moveStates currentState = moveStates.IDLE;

    PIDController latPid = new PIDController(LAT_KP, LAT_KI, LAT_KD);
    PIDController vertPid = new PIDController(VERT_KP, VERT_KI, VERT_KD);
    PIDController turnPid = new PIDController(TURN_KP, TURN_KI, TURN_KD);

    private ElapsedTime timer = new ElapsedTime();


    public void step() {
        if(profile == null){
        // get current position
        EditablePose2D currPos = this.getDrivetrain().getCurrPos();
        // get current time
        double currentTime = timer.seconds();
        // get where we want to go

        // set wheel powers
    }

    public void reset() {
        timer.reset();
        latPid.reset();
        vertPid.reset();
        turnPid.reset();
    }
}