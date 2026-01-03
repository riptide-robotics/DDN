package org.firstinspires.ftc.teamcode.Modules;

import static org.firstinspires.ftc.teamcode.riptideUtil.FORWARD_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.POINT_TOLERANCE;
import static org.firstinspires.ftc.teamcode.riptideUtil.START_POSITION;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KD;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.TURN_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.FORWARD_KI;
import static org.firstinspires.ftc.teamcode.riptideUtil.FORWARD_KP;
import static org.firstinspires.ftc.teamcode.riptideUtil.shortestAngleDiff;
import static java.lang.Thread.sleep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories.LinearTrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.Modules.Utils.EditablePose2D;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Placeholder;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// ----- READY TO TRANSFER ----- //

public class Drivetrain {

    /// ///////////////////////////////////////////
    /// /                                     /////
    /// /              VARIABLES              /////
    /// /                                     /////
    /// ///////////////////////////////////////////

    // -------- DRIVETRAIN MOTORS -------- //
    private final DcMotor frWheel, flWheel, brWheel, blWheel;
    private final GoBildaPinpointDriver pinpoint;
    private final IMU imu;
    private ElapsedTime timer;

    private final OdometryLocalizer robotPos;
    private final double xOdoOffsetInInches = 2.0;
    private final double yOdoOffsetInInches = 4.0;

    // -------- AUTONOMOUS CONTROLLERS -------- //

    private final PIDController turnController = new PIDController(TURN_KP, TURN_KI, TURN_KD);
    private final PIDController forwardController = new PIDController(FORWARD_KP, FORWARD_KI, FORWARD_KD);
    // !!! ADD PATH HERE !!! //
    private final Trajectory path = new LinearTrajectoryBuilder()
            .moveTo(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90))
            .moveTo(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90))
            .build();


    /// ///////////////////////////////////////////
    /// /                                     /////
    /// /              FUNCTIONS              /////
    /// /                                     /////
    /// ///////////////////////////////////////////

    // --------- INITIALIZATION --------- //
    public Drivetrain(HardwareMap hardwareMap) {

        frWheel = hardwareMap.dcMotor.get("frWheel");
        flWheel = hardwareMap.dcMotor.get("flWheel");
        brWheel = hardwareMap.dcMotor.get("brWheel");
        blWheel = hardwareMap.dcMotor.get("blWheel");

        blWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        frWheel.setDirection(DcMotorSimple.Direction.FORWARD);
        brWheel.setDirection(DcMotorSimple.Direction.FORWARD);

        brWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        blWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        blWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        brWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setOffsets(xOdoOffsetInInches, yOdoOffsetInInches, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        robotPos = new OdometryLocalizer(pinpoint, 10);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.resetYaw();
        imu.initialize(parameters);

    }

    // ----------- START/STOP ----------- //

    public void resetImu() {
        imu.resetYaw();
    }

    // ------------ SETTERS ------------ //
    public void setWheelPowers(double flWheelPower, double frWheelPower, double brWheelPower, double blWheelPower) {
        frWheel.setPower(frWheelPower);
        flWheel.setPower(flWheelPower);
        brWheel.setPower(brWheelPower);
        blWheel.setPower(blWheelPower);
    }

    public void setForwardController(double kp, double ki, double kd) {
        forwardController.setPID(kp, ki, kd);
        forwardController.reset();
    }

    public void setTurnController(double kp, double ki, double kd) {
        turnController.setPID(kp, ki, kd);
        forwardController.reset();
    }

    public void resetCurrPos(){
       getPinpoint().resetPosAndIMU();
       getPinpoint().setPosition(START_POSITION);
    }

    // ------------ GETTERS ------------ //
    public Pose2D getCurrPos() {
        return robotPos.getCurrPos();
    }

    public void startOdometry() {
        Thread localizer = new Thread(robotPos);
        localizer.start();
    }

    public OdometryLocalizer getRobotPos() {
        return robotPos;
    }

    public double getRobotHeading(AngleUnit unit) {
        return imu.getRobotYawPitchRollAngles().getYaw(unit); // heading of bot in radians
    }

    public GoBildaPinpointDriver getPinpoint() {
        return pinpoint;
    }

    public double[] getWheelPowers() {
        return new double[]{flWheel.getPower(), frWheel.getPower(), blWheel.getPower(), brWheel.getPower()};
    }

    // -------- Methods --------------- //
    // boolean for testing if it got to the point, temporary
    public boolean followGivenPath(double time) {
        Trajectory.PathSample goal = path.getExpectedPosition(time);
        boolean atPoint = goToPosPID(new Pose2D(DistanceUnit.INCH, goal.x, goal.y, AngleUnit.RADIANS, goal.heading));
//       if (atPoint){
//           //do some stuff maybe telemetry stuff
//       }
        return atPoint;
    }

    public boolean goToPosPID(Pose2D goal) {
        pinpoint.update();
        double dx = goal.getX(DistanceUnit.INCH) - pinpoint.getPosX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH) - pinpoint.getPosY(DistanceUnit.INCH);
        double distanceToPoint = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        double headingError = shortestAngleDiff(this.getRobotHeading(AngleUnit.RADIANS), Math.atan2(dy, dx));

        double headingx = Math.cos(getRobotHeading(AngleUnit.RADIANS));
        double headingy = Math.sin(getRobotHeading(AngleUnit.RADIANS));
        double[] headingVect = {headingx, headingy};
        double[] pointVect = {dx / distanceToPoint, dy / distanceToPoint};
        double fbError = headingVect[0] * pointVect[0] + headingVect[1] * pointVect[1];

        double forwardPower = forwardController.calculate(0, fbError);
        double turnPower = turnController.calculate(0, headingError);

        this.setWheelPowers(
                forwardPower - turnPower,
                forwardPower + turnPower,
                forwardPower + turnPower,
                forwardPower - turnPower
        );

        return atPoint(goal);
    }

    public boolean atPoint(Pose2D point) {
        double dx = point.getX(DistanceUnit.INCH) - getCurrPos().getX(DistanceUnit.INCH);
        double dy = point.getY(DistanceUnit.INCH) - getCurrPos().getY(DistanceUnit.INCH);

        return Math.sqrt(dx * dx + dy * dy) < POINT_TOLERANCE;
    }
}