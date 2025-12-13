    package org.firstinspires.ftc.teamcode.Teleop;

    import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_BOT;
    import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_TOP;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_RESTING;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_UP;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.teamcode.Modules.Intake;
    import org.firstinspires.ftc.teamcode.Robot;


    @Config
    @TeleOp(name = "Meet 2 FSM Manual")
    public class Meet2FSMManual extends LinearOpMode {
        Robot robot;


        boolean hasrun = false;
        boolean recieve = false;
        boolean outtake = false;
        boolean runIntakePos = true;
        boolean runOuttakePos = true;
        public static  double spin = 0.85;

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

        /**            OUTTAKE              **/
        public static double currentTopRPMGoal;
        public static double currentBottomRPMGoal;



        /**            TIMERS              **/
        ElapsedTime bootKickerDelayTimerUp;
        ElapsedTime bootKickerDelayTimerDown;
        ElapsedTime bootKickActivateDelayTimer;
        ElapsedTime spindexDelayTimer;
        ElapsedTime endTimer;
        public static double spindexDelay = 1000; // IN MS
        public static double bootKickDelay = 300; // IN MS
        public static double bootKickActivateDelay = 1500;

        boolean didRumble = false;
        boolean startedDelay = false;
        boolean moveToNextOuttakeSlot = false;



        /**            DEBOUNCE              **/
        boolean rightBumperPressedG2 = false;
        boolean backPressedG2 = false;
        boolean leftBumperPressedG2 = false;
        boolean leftTriggerPressedG2 = false;
        boolean xPressedG2 = false;
        boolean rightPressedG2 = false;
        boolean yPressedG2 = false;



        @Override
        public void runOpMode() throws InterruptedException {
            hasrun = false;
            robot = new Robot(hardwareMap);
            bootKickerDelayTimerUp = new ElapsedTime();
            bootKickerDelayTimerDown = new ElapsedTime();
            spindexDelayTimer = new ElapsedTime();
            bootKickActivateDelayTimer= new ElapsedTime();
            endTimer = new ElapsedTime();
            recieve = true;
            outtake = false;
            runIntakePos = false;
            runOuttakePos = false;
            moveToNextOuttakeSlot = false;

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

            bootKickActivateDelayTimer.reset();
            bootKickActivateDelayTimer.startTime();
            telemetry.clear();
            telemetry.addData("Robot status", "Started!");
            telemetry.update();

            robot.getOuttake().startFlywheel();

            while(opModeIsActive()){

                FSM();
                tankDrive();
                robot.getOuttake().runOuttakePID(currentTopRPMGoal, currentBottomRPMGoal, tele);

                double currTime = endTimer.seconds();
                mapJoyToAngle(gamepad2.right_stick_x);
                robot.getOuttake().
                if (currTime >= 80 && !didRumble){gamepad1.rumble(1, 1, 500); gamepad2.rumble(1, 1, 500);}
                if (currTime >= 82) {didRumble = true;}
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

                    // INTAKE
                    if (gamepad2.right_trigger >= 0.1) {
                        robot.getIntake().spin(0.6);
                    } else if (gamepad2.back) {
                        robot.getIntake().spin(-0.6);
                    } else {
                        robot.getIntake().spin(0);
                    }

                    cycleSlots();

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
        }

        public void tankDrive() {
            double slowdown = gamepad1.right_trigger > 0 ? 0.25 : 1;
            robot.getDrivetrain().setWheelPowers(
                    -gamepad1.left_stick_y * slowdown,
                    -gamepad1.right_stick_y * slowdown,
                    -gamepad1.right_stick_y * slowdown,
                    -gamepad1.left_stick_y * slowdown
            );
        }

        public void cycleSlots(){
            if (gamepad2.y && !yPressedG2 && !recieve) {
                currentTopRPMGoal = 0;
                currentBottomRPMGoal = 0;
//                CYCLE MANUALLY
//                if (spindexPosIntake == -1) spindexPosIntake = 0;
//                else if (spindexPosIntake == 0) spindexPosIntake = 1;
//                else if (spindexPosIntake == 1) spindexPosIntake = 2;
//                else if (spindexPosIntake == 2) spindexPosIntake = 0;
                outtake = false;
                recieve = true;
                spindexPosIntake = robot.getIntake().getNextIntakeSlot();
                yPressedG2 = true;
            }

            if (gamepad2.y && !yPressedG2 && recieve){
                currentTopRPMGoal = 0;
                currentBottomRPMGoal = 0;
                recieve = false;
                yPressedG2 = true;
            }
            robot.getIntake().spin(spin);
            if (recieve){
                //              CYCLE AUTOMATIC
                spindexPosOuttake = -1;
                if (spindexPosIntake != -1 && !runIntakePos) {
                    if (spindexPosIntake == 0) {
                        if (Intake.SLOT_0 == Intake.slotStatus.BLANK) {
                            Intake.SLOT_0 = robot.getIntake().currColor();
                            runIntakePos = true;
                        }
                    }

                    if (spindexPosIntake == 1) {
                        if (Intake.SLOT_1 == Intake.slotStatus.BLANK) {
                            Intake.SLOT_1 = robot.getIntake().currColor();
                            runIntakePos = true;
                        }
                    }

                    if (spindexPosIntake == 2) {
                        if (Intake.SLOT_2 == Intake.slotStatus.BLANK) {
                            Intake.SLOT_2 = robot.getIntake().currColor();
                            runIntakePos = true;
                        }
                    }
                }
                if (runIntakePos) {
                    spindexPosIntake = robot.getIntake().getNextIntakeSlot();
                    if (spindexPosIntake == 0) {
                        robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_RECEIVE);
                    } else if (spindexPosIntake == 1) {
                        robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_1_RECEIVE);
                    } else if (spindexPosIntake == 2) {
                        robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_2_RECEIVE);
                    }

                    if (!startedDelay) {
                        spindexDelayTimer.reset();
                        startedDelay = true;
                    }

                    if (spindexDelayTimer.milliseconds() >= spindexDelay) {
                        tele.addData("spindex delay active ", spindexDelayTimer.milliseconds());
                        runIntakePos = false;
                        startedDelay = false;
                    }
                }
                tele.addLine("Spindex is Receiving");
                tele.addData("RunIntakePos: ", runIntakePos);
            }

            if (gamepad2.x && !xPressedG2 && !outtake) {
                outtake = true;
                recieve = false;
                currentTopRPMGoal = MID_DIST_TOP;
                currentBottomRPMGoal = MID_DIST_BOT;
                spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
                xPressedG2 = true;
            }

            if (gamepad2.x && !xPressedG2 && outtake){
                currentTopRPMGoal = 0;
                currentBottomRPMGoal = 0;
                outtake = false;
                xPressedG2 = true;
            }

            if (outtake){
                spindexPosIntake = -1;

                if (spindexPosOuttake != -1){
                    if (spindexPosOuttake == 0) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_SHOOT);}

                    if (spindexPosOuttake == 1) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_1_SHOOT);}

                    if (spindexPosOuttake == 2) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_2_SHOOT);}
                }

                if (gamepad2.a && robot.getOuttake().isAtGoalSpeed() && !moveToNextOuttakeSlot) {
                    if (bootKickActivateDelayTimer.milliseconds() >= bootKickActivateDelay) {
                        robot.getIntake().bootkick(SPINDEX_ARM_UP);
                        bootKickerDelayTimerUp.reset();
                        bootKickerDelayTimerUp.startTime();
                        tele.addLine("Boot kick activate delay");
                    }
                }

                if (robot.getIntake().bootKickCurrPos() == SPINDEX_ARM_UP && !moveToNextOuttakeSlot) {
                    if (bootKickerDelayTimerUp.milliseconds() >= bootKickDelay) {
                        robot.getIntake().bootkick(SPINDEX_ARM_RESTING);
                        bootKickerDelayTimerDown.reset();
                        bootKickerDelayTimerDown.startTime();
                        moveToNextOuttakeSlot = true;
                        tele.addLine("Boot kick up delay");
                    }
                }

                if (robot.getIntake().bootKickCurrPos() == SPINDEX_ARM_RESTING && moveToNextOuttakeSlot){
                    if (bootKickerDelayTimerDown.milliseconds() >= bootKickDelay){
                        if (spindexPosOuttake == 0){Intake.SLOT_0 = Intake.slotStatus.BLANK;}
                        else if (spindexPosOuttake == 1){Intake.SLOT_1 = Intake.slotStatus.BLANK;}
                        else if (spindexPosOuttake == 2){Intake.SLOT_2 = Intake.slotStatus.BLANK;}
                        spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();
                        bootKickActivateDelayTimer.reset();
                        bootKickActivateDelayTimer.startTime();
                        moveToNextOuttakeSlot = false;
                        tele.addLine("Boot kick down delay");
                    }
                }

                tele.addLine("Spindex is outtaking");
            }

            tele.addData("Intake Slot: ", spindexPosIntake);
            tele.addData("Outtake Slot: ", spindexPosOuttake);
            tele.addData("Slot 0: ", Intake.SLOT_0);
            tele.addData("Slot 1: ", Intake.SLOT_1);
            tele.addData("Slot 2: ", Intake.SLOT_2);
            tele.addData("Current Status: ", robot.getIntake().currColor());
            tele.addData("Current Color: ", robot.getIntake().checkColor());

        }
    }