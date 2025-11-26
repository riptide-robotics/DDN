    package org.firstinspires.ftc.teamcode.Teleop;

    import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_BOT;
    import static org.firstinspires.ftc.teamcode.riptideUtil.MID_DIST_TOP;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_ONE_PIKCUP_POS;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_ONE_SHOOT_POS;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_THREE_PIKCUP_POS;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_THREE_SHOOT_POS;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_TWO_PIKCUP_POS;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SLOT_TWO_SHOOT_POS;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_RESTING;
    import static org.firstinspires.ftc.teamcode.riptideUtil.SPINDEX_ARM_UP;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
    import org.firstinspires.ftc.teamcode.Modules.Intake;
    import org.firstinspires.ftc.teamcode.Robot;
    import org.firstinspires.ftc.teamcode.UnitTests.SpindexPositions;


    @Config
    @TeleOp(name = "Meet 2 FSM Manual")
    public class Meet2FSMManual extends LinearOpMode {
        Robot robot;

        boolean hasrun = false;
        boolean updateTime = false;
        boolean reset = false;
        boolean align = false;

        ElapsedTime endTimer = new ElapsedTime();

        boolean runOuttake = false;

        public static double currentTopRPMGoal;
        public static double currentBottomRPMGoal;

        public static double spindexPosIntake = 1;
        public static double spindexPosOuttake = -1;

        public static double delay = 500; // IN MS

        ElapsedTime timer;

        boolean didRumble = false;


        // DEBOUNCE
        boolean rightBumperPressedG2 = false;
        boolean backPressedG2 = false;
        boolean leftBumperPressedG2 = false;
        boolean leftTriggerPressedG2 = false;
        boolean xPressedG2 = false;
        boolean rightPressedG2 = false;
        boolean yPressedG2 = false;

        Telemetry tele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        public enum states{
            TELEOP,
            ENDGAME
        }

        public static double spindexPos = 0;
        public states currentState = states.TELEOP;

        @Override
        public void runOpMode() throws InterruptedException {
            hasrun = false;
            robot = new Robot(hardwareMap);
            timer = new ElapsedTime();

            robot.getIntake().initSpindex();

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
                //cycleSlots();

                //telemetry.addData("Angle: ", robot.getDrivetrain().getRobotHeading(AngleUnit.DEGREES));
    //
    //            if (updateTime){
    //                robot.getOuttake().startFlywheel();
    //            }
                robot.getOuttake().runOuttakePID(currentTopRPMGoal, currentBottomRPMGoal, tele);

                double currTime = endTimer.seconds();

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
                    toggleBootKicker();

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

        private void fieldCentricDrive() {
            double slowdown = gamepad1.right_trigger > 0 ? 0.25 : 1;
            double y = -gamepad1.left_stick_y * slowdown;
            double x = gamepad1.left_stick_x * 1.1 * slowdown;
            double rx = gamepad1.right_stick_x * slowdown;

            double heading = robot.getDrivetrain().getRobotHeading(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
            double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frWheelPower = (rotY - rotX - rx) / denominator;
            double flWheelPower = (rotY + rotX + rx) / denominator;
            double brWheelPower = (rotY + rotX - rx) / denominator;
            double blWheelPower = (rotY - rotX + rx) / denominator;

            robot.getDrivetrain().setWheelPowers(flWheelPower, frWheelPower, brWheelPower, blWheelPower);

            if (gamepad1.y) {
                robot.getDrivetrain().resetImu();
            }
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
            if (gamepad2.y && !yPressedG2) {
//                CYCLE MANUALLY
//                if (spindexPosIntake == -1) spindexPosIntake = 0;
//                else if (spindexPosIntake == 0) spindexPosIntake = 1;
//                else if (spindexPosIntake == 1) spindexPosIntake = 2;
//                else if (spindexPosIntake == 2) spindexPosIntake = 0;

//              CYCLE AUTOMATIC
                spindexPosIntake = robot.getIntake().getNextIntakeSlot();

                spindexPosOuttake = -1;

                // REPlACE LATER WITH ACTUAL COLOR AND CHECK WITH COLOR SENSOR TO SEE IF BALL IS ACTUALLY IN SPINDEX
                if (spindexPosIntake != -1) {
                    if (spindexPosIntake == 0) robot.getIntake().SLOT_0 = Intake.slotStatus.PURPLE;
                    else if (spindexPosIntake == 1) robot.getIntake().SLOT_1 = Intake.slotStatus.PURPLE;
                    else if (spindexPosIntake == 2) robot.getIntake().SLOT_2 = Intake.slotStatus.PURPLE;
                }

                if (spindexPosIntake == 0) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_RECEIVE);}
                else if (spindexPosIntake == 1) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_1_RECEIVE);}
                else if (spindexPosIntake == 2) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_2_RECEIVE);}

                yPressedG2 = true;
            }

            if (gamepad2.x && !xPressedG2) {
//                CYCLE MANUALLY
//                if (spindexPosOuttake == -1) spindexPosOuttake = 0;
//                else if (spindexPosOuttake == 0) spindexPosOuttake = 1;
//                else if (spindexPosOuttake == 1) spindexPosOuttake = 2;
//                else if (spindexPosOuttake == 2) spindexPosOuttake = 0;
//            spindexPosOuttake = getNextOuttakeSlot();

//              CYCLE AUTOMATIC
                spindexPosOuttake = robot.getIntake().getNextOuttakeSlot();

                if (spindexPosOuttake != -1) {
                    if (spindexPosOuttake == 0) robot.getIntake().SLOT_0 = Intake.slotStatus.BLANK;
                    else if (spindexPosOuttake == 1) robot.getIntake().SLOT_1 = Intake.slotStatus.BLANK;
                    else if (spindexPosOuttake == 2) robot.getIntake().SLOT_2 = Intake.slotStatus.BLANK;
                }

                if (spindexPosOuttake == 0) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_0_SHOOT);}
                else if (spindexPosOuttake == 1) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_1_SHOOT);}
                else if (spindexPosOuttake == 2) {robot.getIntake().goTo(Intake.UnshiftedPositions.SLOT_2_SHOOT);}

                spindexPosIntake = -1;



                xPressedG2 = true;
            }

            tele.addData("Intake Slot: ", spindexPosIntake);
            tele.addData("Outtake Slot: ", spindexPosOuttake);
            tele.addData("Slot 0: ", robot.getIntake().SLOT_0);
            tele.addData("Slot 1: ", robot.getIntake().SLOT_1);
            tele.addData("Slot 2: ", robot.getIntake().SLOT_2);
        }

        public void toggleBootKicker(){
            if (robot.getIntake().spindexCurrentPosition() == Intake.UnshiftedPositions.SLOT_0_SHOOT.posUnshifted || robot.getIntake().spindexCurrentPosition() == Intake.UnshiftedPositions.SLOT_1_SHOOT.posUnshifted || robot.getIntake().spindexCurrentPosition() == Intake.UnshiftedPositions.SLOT_2_SHOOT.posUnshifted) {
                if (gamepad2.a && robot.getOuttake().isAtGoalSpeed()) {
                    robot.getIntake().BootKick(SPINDEX_ARM_UP);
                    tele.addLine("Boot Kicker Up");
                    timer.reset();
                    timer.startTime();
                }
            }

            if (robot.getIntake().bootKickCurrPos() == SPINDEX_ARM_UP) {
                if (timer.milliseconds() >= delay) {robot.getIntake().BootKick(SPINDEX_ARM_RESTING);}
            }
        }
    }