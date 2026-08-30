package org.firstinspires.ftc.teamcode.Modules;

// READY TO TRANSFER

import static org.firstinspires.ftc.teamcode.riptideUtil.*;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories.LinearTrajectoryBuilder;
import org.firstinspires.ftc.teamcode.Autonomous.Utils.Trajectories.Trajectory;
import org.firstinspires.ftc.teamcode.Modules.Utils.GoBildaPinpointDriver;

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
    private final double xOdoOffsetInInches = 4.1;
    private final double yOdoOffsetInInches = 3.8;

    // -------- AUTONOMOUS CONTROLLERS -------- //

    private final PIDController turnControllerCW = new PIDController(TURN_KP_CW, TURN_KI_CW, TURN_KD_CW);
    private final PIDController turnControllerCCW = new PIDController(TURN_KP_CCW, TURN_KI_CCW, TURN_KD_CCW);
    private final PIDController forwardControllerFar = new PIDController(FORWARD_KP_FAR, FORWARD_KI_FAR, FORWARD_KD_FAR);
    private final PIDController forwardControllerClose = new PIDController(FORWARD_KP_CLOSE, FORWARD_KI_CLOSE, FORWARD_KD_CLOSE);
    // !!! ADD PATH HERE !!! //
    private final Trajectory path = new LinearTrajectoryBuilder()
            .moveTo(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90))
            .moveTo(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90))
            .build();
    private double[] wheelPowers = new double[4];
    private double fbPower;
    private double fbDist;
    private double fbVectNorm;
    private double anglePower;
    private double angleDiff;
    private double angleATan;
    private boolean tuningFB = false;
    private boolean tuningAngle = false;


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

        frWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setOffsets(xOdoOffsetInInches, yOdoOffsetInInches, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        robotPos = new OdometryLocalizer(pinpoint, 10);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.resetYaw();
        imu.initialize(parameters);

        pinpoint.setPosition(START_POSITION);

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

    public void setForwardControllerFar(double kp, double ki, double kd) {
        forwardControllerFar.setPID(kp, ki, kd);
        forwardControllerFar.reset();
    }

    public void setForwardControllerClose(double kp, double ki, double kd) {
        forwardControllerClose.setPID(kp, ki, kd);
        forwardControllerClose.reset();
    }

    public void setTurnControllerCW(double kp, double ki, double kd) {
        turnControllerCW.setPID(kp, ki, kd);
        forwardControllerFar.reset();
    }

    public void setTurnControllerCCW(double kp, double ki, double kd) {
        turnControllerCCW.setPID(kp, ki, kd);
        forwardControllerFar.reset();
    }

//    public void setPinpointPos(Pose2D pos) {
//        pinpoint.setPosition(pos);
//        pinpoint.setHeading(pos.getHeading(AngleUnit.DEGREES), AngleUnit.DEGREES);
//    }

//    public void resetCurrPos(){
//       //getPinpoint().resetPosAndIMU();
//       setPinpointPos(START_POSITION);
//    }

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

    public double[] getWheelPowersArray() {
        return wheelPowers;
    }

    public double getFbPower() {
        return fbPower;
    }

    public double getFbDist() {
        return fbDist;
    }

    public double getFbVectNorm() {
        return fbVectNorm;
    }

    public double getAngleDiff() {
        return angleDiff;
    }

    public double getAnglePower() {
        return anglePower;
    }

    public double getAngleATan() {
        return angleATan;
    }

    // -------- Methods --------------- //

    public void tuningFB() {
        tuningFB = true;
    }

    public void tuningAngle() {
        tuningAngle = true;
    }

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
        return this.goToPosPID(goal, FtcDashboard.getInstance().getTelemetry());
    }

    public boolean goToPosPID(Pose2D goal, Telemetry t) {
        pinpoint.update();
        if (atPoint(goal)) {
            this.setWheelPowers(0, 0, 0, 0);
            return true;
        }
        double dx = goal.getX(DistanceUnit.INCH) - pinpoint.getPosX(DistanceUnit.INCH);
        double dy = goal.getY(DistanceUnit.INCH) - pinpoint.getPosY(DistanceUnit.INCH);
        double distanceToPoint = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

        double headingError = shortestAngleDiff(getCurrPos().getHeading(AngleUnit.DEGREES), Math.toDegrees(Math.atan2(dy, dx)));

        angleATan = Math.toDegrees(Math.atan2(dy, dx));

        double headingx = Math.cos(getCurrPos().getHeading(AngleUnit.RADIANS)); // William you jerk
        double headingy = Math.sin(getCurrPos().getHeading(AngleUnit.RADIANS)); // These are supposed to be in radians dummy
        double[] headingVect = {headingx, headingy};
        double[] pointVect = {dx / distanceToPoint, dy / distanceToPoint};
        double dotProd = headingVect[0] * pointVect[0] + headingVect[1] * pointVect[1];
        dotProd = Math.pow(dotProd, 2);
        // I was typing this line out and was like; wait a minute, something ain't right
        //double dotProd = 1 * Math.sqrt(pointVect[0] * pointVect[0] + pointVect[1] * pointVect[1]) * Math.cos(Math.toRadians(headingError));
        double fbError = distanceToPoint * (dotProd);

        fbPower = fbError;
        fbDist = distanceToPoint;
        fbVectNorm = dotProd;

        angleDiff = headingError;

        double forwardPower = (fbError >= FB_CLOSE_THRESHOLD) ? forwardControllerFar.calculate(0, fbError) : forwardControllerClose.calculate(0, fbError);
        // to avoid deathspirals
        // A little overtuned rn have to work out how to make forward power scale based on heading more.
        //double alignmentScalar = Math.max(0, Math.cos(headingError));

        //forwardPower *= Math.pow(alignmentScalar, 2);

        t.addData("Forward Power", forwardPower);

        double turnPower = (headingError >= 0) ? turnControllerCCW.calculate(0, headingError) : turnControllerCW.calculate(0, headingError);
        t.addData("Turn Power", turnPower);
        if (distanceToPoint <= TURN_THRESHOLD) {
            turnPower = 0;
        }

        anglePower = turnPower;

        if(tuningFB) {turnPower = 0;}
        if(tuningAngle) {forwardPower = 0;}

        wheelPowers[0] = forwardPower - turnPower;
        wheelPowers[1] = forwardPower + turnPower;
        wheelPowers[2] = forwardPower + turnPower;
        wheelPowers[3] = forwardPower - turnPower;

        double maxWheelPower = Math.max(Math.abs(forwardPower) + Math.abs(turnPower), 1);

        this.setWheelPowers(
                (forwardPower - turnPower) / maxWheelPower,
                (forwardPower + turnPower) / maxWheelPower,
                (forwardPower + turnPower) / maxWheelPower,
                (forwardPower - turnPower) / maxWheelPower
        );

        return atPoint(goal);
    }

    public boolean turnOnPointPID(double goal) {
        pinpoint.update();
        double headingError = shortestAngleDiff(getCurrPos().getHeading(AngleUnit.DEGREES), goal);
        double turnPower = (headingError >= 0) ? turnControllerCCW.calculate(0, headingError) : turnControllerCW.calculate(0, headingError);
        this.setWheelPowers(
                -turnPower,
                turnPower,
                turnPower,
                -turnPower
        );
        return headingError < 5;
    }

    public boolean atPoint(Pose2D point) {
        double dx = point.getX(DistanceUnit.INCH) - getCurrPos().getX(DistanceUnit.INCH);
        double dy = point.getY(DistanceUnit.INCH) - getCurrPos().getY(DistanceUnit.INCH);

        return Math.sqrt(dx * dx + dy * dy) < POINT_TOLERANCE;
    }
}
