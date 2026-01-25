    package org.firstinspires.ftc.teamcode.Teleop;

    import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_BOT;
    import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_TOP;
    import static org.firstinspires.ftc.teamcode.riptideUtil.BOOT_KICKER_RESTING;
    import static org.firstinspires.ftc.teamcode.riptideUtil.BOOT_KICKER_UP;
    import static org.firstinspires.ftc.teamcode.riptideUtil.moveToNextSlot;
    import static org.firstinspires.ftc.teamcode.riptideUtil.nextShotAvailable;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
    import org.firstinspires.ftc.teamcode.Modules.Intake;
    import org.firstinspires.ftc.teamcode.Robot;


    @Config
    @TeleOp(name = "Meet 2 FSM Manual")
    public class Meet2FSMManual extends LinearOpMode {
        Robot robot;


        boolean hasrun = false;
        boolean recieve = false;
        boolean outtake = false;
        boolean runOuttakePos = true;
        public static double spin = 1;

        Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        public enum states{
            TELEOP,
            ENDGAME
        }
        public states currentState = states.TELEOP;

        /**            SPINDEX              **/
        public static double spindexPosIntake = 1;
        public static double spindexPosOuttake = -1;
        public char currColor = 'b';
        boolean resetBootKicker;
        boolean idleSpinOverride = false;

        /**            OUTTAKE              **/
        public static double currentTopRPMGoal;
        public static double currentBottomRPMGoal;



        /**            TIMERS               **/
        ElapsedTime spindexDelayTimer;
        ElapsedTime endTimer;
        ElapsedTime scanDelayTimer;
        public static double spindexDelay = 500; // IN MS
        boolean resetBootKick = false;
        public static double scanDelay = 500;

        boolean didRumble = false;
        boolean startedDelay = false;
        boolean moveToNextOuttakeSlot = false;



        /**            DEBOUNCE             **/
        boolean rightBumperPressedG2 = false;
        boolean backPressedG2 = false;
        boolean leftBumperPressedG2 = false;
        boolean leftTriggerPressedG2 = false;
        boolean xPressedG2 = false;
        boolean rightPressedG2 = false;
        boolean yPressedG2 = false;
        boolean bPressedG2 = false;
        boolean aPressedG2 = false;
        boolean dUpPressedG2 = false;
        boolean dDownPressedG2 = false;
        boolean isOuttakeOn = false;



        @Override
        public void runOpMode() throws InterruptedException {
            hasrun = false;
            robot = new Robot(hardwareMap);
            spindexDelayTimer = new ElapsedTime();
            endTimer = new ElapsedTime();
            scanDelayTimer = new ElapsedTime();
            recieve = true;
            outtake = false;
            moveToNextSlot = false;
            runOuttakePos = false;
            moveToNextOuttakeSlot = false;
            resetBootKick = false;
            nextShotAvailable = true;
            isOuttakeOn = false;
            spindexPosIntake = robot.getIntake().getNextIntakeSlot();

            robot.getIntake().initSpindex();
            robot.getIntake().initColorSensor();

            telemetry.addData("Robot status:", "succesfully initiated");
            telemetry.update();

            currentTopRPMGoal = 0;
            currentBottomRPMGoal = 0;

            waitForStart();
            if (isStopRequested()) return;
            endTimer.reset();
            endTimer.startTime();

            telemetry.clear();
            telemetry.addData("Robot status", "Started!");
            telemetry.update();

            robot.getOuttake().startFlywheel();

            while(opModeIsActive()){

                FSM();
                tankDrive();
                robot.getOuttake().runOuttakePID(currentTopRPMGoal,currentBottomRPMGoal,tele);
                robot.s.loop();
                robot.setStatus((byte) Intake.ballsShot);

//                double currTime = endTimer.seconds();
//                robot.getOuttake().mapJoyToAngle(gamepad2.right_stick_x);
                robot.getOuttake().updateTurntableAngle(tele);
                tele.update();
            }
        }


        private void FSM(){
            switch(currentState) {
                case TELEOP:
                    if (!hasrun) {
                        currentTopRPMGoal = 0;
                        currentBottomRPMGoal = 0;
                        hasrun = true;
                    }

                    cycleSlots();
                    count();

                    // ENDGAME
    //                if (gamepad2.dpad_up){
    //                    currentState = states.ENDGAME;
    //                    hasrun = false;
    //                }

                    break;
                case ENDGAME:
                    if (!hasrun){
                        robot.getEndgameServos().lift();
                        hasrun = true;
                    }

                    if (gamepad2.dpad_down){
                        robot.getEndgameServos().lower();
                    }

                    if (gamepad2.dpad_up){
                        hasrun = false;
                    }

                    if (gamepad2.b){
                        currentState = states.TELEOP;
                        robot.getEndgameServos().lower();
                        hasrun = false;
                    }

                    break;
            }
            if (!gamepad2.dpad_right){rightPressedG2 = false;}
            if (!gamepad2.right_bumper){rightBumperPressedG2 = false;}
            if (!gamepad2.left_bumper){leftBumperPressedG2 = false;}
            if (!(gamepad2.left_trigger > 0.1)){leftTriggerPressedG2 = false;}
            if (!gamepad2.x){xPressedG2 = false;}
            if (!gamepad2.y){yPressedG2 = false;}
            if (!gamepad2.back){backPressedG2 = false;}
            if (!gamepad2.dpad_up) {dUpPressedG2 = false;}
            if (!gamepad2.dpad_down) {dDownPressedG2 = false;}
            if (!gamepad2.a){aPressedG2 = false;}
        }

        public void tankDrive() {
            double slowdown = gamepad1.right_trigger > 0 ? 0.25 : 1;
            robot.getDrivetrain().setWheelPowers(
                    gamepad1.left_stick_y * slowdown,
                    gamepad1.right_stick_y * slowdown,
                    -gamepad1.right_stick_y * slowdown,
                    -gamepad1.left_stick_y * slowdown
            );
        }


        public void count(){
            if (gamepad2.dpad_up && !dUpPressedG2){
                robot.getIntake().increaseCount();
                dUpPressedG2 = true;
            }

            if (gamepad2.dpad_down && !dDownPressedG2){
                robot.getIntake().decreseCount();
                dDownPressedG2 = true;
            }

            if (gamepad2.b && !bPressedG2){
                robot.getIntake().resetCount();
                bPressedG2 = true;
            }
            tele.addData("Current Count: ", robot.getIntake().getCount());
        }

        public void cycleSlots(){
            if (gamepad2.y && !yPressedG2 && !recieve) {
                scanDelayTimer.reset();
                scanDelayTimer.startTime();

                isOuttakeOn = false;
                currentTopRPMGoal = 0;
                currentBottomRPMGoal = 0;
                outtake = false;
                recieve = true;

                spindexPosIntake = robot.getIntake().getNextIntakeSlot();
                yPressedG2 = true;
            }

            if (recieve){
                if (scanDelayTimer.milliseconds() >= scanDelay){
                    robot.getIntake().setSlotColor(spindexPosIntake);
                }
                spindexPosOuttake = -1;
                if (moveToNextSlot) {
                    spindexPosIntake = robot.getIntake().getNextIntakeSlot();
                    robot.getIntake().moveToNextIntakeSlot(spindexPosIntake);
                    robot.getIntake().delayMovementToNextSlot(spindexDelayTimer, spindexDelay, tele);
                }

                if (gamepad2.back){
                    robot.getIntake().spin(-1);
                } else{
                    robot.getIntake().spin(spin);
                }

                tele.addLine("Spindex is Receiving");
                tele.addData("moveToNextSlot: ", moveToNextSlot);
            }

            if (gamepad2.x && !xPressedG2 && !outtake && !isOuttakeOn) {
                outtake = true;
                recieve = false;
                currentTopRPMGoal = MID_DIST_TOP;
                currentBottomRPMGoal = MID_DIST_BOT;
                spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
                xPressedG2 = true;
                isOuttakeOn = true;
            }

            if (gamepad2.x && !xPressedG2 && isOuttakeOn){
                currentTopRPMGoal = 0;
                currentBottomRPMGoal = 0;
                outtake = true;
                xPressedG2 = true;
                isOuttakeOn = false;
            }

            if (outtake){
                robot.getIntake().spin(0);
                spindexPosIntake = -1;
                robot.getIntake().cycleOuttakeSlot(spindexPosOuttake);
                if (gamepad2.a && !aPressedG2 && nextShotAvailable) {
                    nextShotAvailable = false;
                    robot.outtake(spindexPosOuttake, tele);
                    aPressedG2 = true;
                }

                if (nextShotAvailable) {
                    spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
                }

                /*************************************************
                                Add turntable stuff here
                 *************************************************/

            }
            tele.addData("Intake Slot: ", spindexPosIntake);
            tele.addData("Outtake Slot: ", spindexPosOuttake);
            tele.addData("Slot 0: ", Intake.SLOT_0);
            tele.addData("Slot 1: ", Intake.SLOT_1);
            tele.addData("Slot 2: ", Intake.SLOT_2);
            tele.addData("Current Color: ", robot.getIntake().checkColor());
            tele.addData("Is at goal speed ", robot.getOuttake().isAtGoalSpeed());
            tele.addData("Next Slot Available ", nextShotAvailable);
        }
    }
