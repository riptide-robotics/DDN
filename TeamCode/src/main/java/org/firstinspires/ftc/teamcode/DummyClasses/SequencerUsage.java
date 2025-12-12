package org.firstinspires.ftc.teamcode.DummyClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;


/*
This class demonstrates the usage off the sequencer system.
This is a repurposed UnitTest that has been used for the testing of said sequencer system.
As a result, it contains all the necessary components of the Sequencer system, making it
helpful for this purpose.

We'll be taking the Sequencer class from Robot. If you want to create your own
(which will run separately) you can simply use the constructor. It takes a drivetrain,
which you can ignore - just put null in there for now.
*/
public class SequencerUsage extends LinearOpMode {
    Robot robot;

    @Override
    public void runOpMode() throws InterruptedException {
        
        robot = new Robot(hardwareMap);
        
        //standard helper methods
        waitForStart();
        if (isStopRequested()) return;

        //yes, we're going to kick up the outtake.
        robot.getOuttake().startFlywheel();

        
        while (opModeIsActive()) {
            //This is where all the interesting stuff will happen.
            sequencerTest();
        }
    }

    
    //These are all about showing the sequencer on its job. You'll be seeing these a lot very soon.
    public boolean hasARun = false;
    public boolean hasBARun = false;
    public boolean hasBBRun = false;
    public int numberXRun = 0;

    public boolean hasALoop = false;
    public boolean hasBALoop = false;
    public boolean hasBBLoop = false;

    public int numberXLoop = 0;
    boolean hasXRun = false;

    private void sequencerTest() {
        //robot.s is a general-purpose sequencer in the robot.
        //Anything specific can be in a specialized Sequencer object.
        
        
        //The simplest of the tests. This one will set actionA in the telemetry to true after one second.
        telemetry.addData("actionA", hasARun);
        if (gamepad1.aWasPressed() && robot.s.impulseactions.isEmpty())
            
            //This will be how actions are added. We start off with a lambda,
            //which contains all the necessary info. After the lambda, we can
            //set the timer. It is in seconds - as the label shouts at us - 
            //so if you want to do anything more precise, you'll have to do it
            //with decimals. Be warned, however; the actions are only processed
            //at the same speed as your loop.
            
            robot.s.addImpulseAction(() -> {
                hasARun = true;
            }, 1);


        
        //The next three actions are an example of how actions can be set to run at the same time.
        telemetry.addData("actionBA", hasBARun);
        telemetry.addData("actionBB", hasBBRun);
        if (gamepad1.bWasPressed() && robot.s.impulseactions.isEmpty()) {
            robot.s.addImpulseAction(() -> {
                hasBARun = true;
            }, 1);
            robot.s.addImpulseAction(() -> {
                hasBBRun = true;
            }, 1);
        }
        
        telemetry.addData("actionXCount", numberXRun);
        if (gamepad1.x) {
            robot.s.addImpulseAction(() -> {
                numberXRun++;
            }, 3);
        }

        /*
        * The next set of actions are loop actions. They specialize constantly repeating an action after a delay.
        *
        * Under the hood, they just aren't removed like an ImpulseAction - so its perfectly fine to kill (remove)
        * them inside their run loop.
        *
        * There are two ways to remove the action, which work off the same principle. You can get their action
        * directly and set killAction to true, but the easier way to do it is to use killLoopAction.
        * Don't use killTLoopAction; that is for something in the future.
        */


        //This is a standard loop. This particular version kills itself immediately, functioning more like an
        //Impulse Action.
        telemetry.addData("loopA", hasALoop);
        if (gamepad2.aWasPressed()) {
            robot.s.addLoopAction(() -> {
                hasALoop = true;
                robot.s.getLoopAction("loopA").killAction = true;
            }, 1, "loopA");
        }

        telemetry.addData("loopBA", hasBALoop);
        telemetry.addData("loopBB", hasBBLoop);

        //Staggering loop actions.
        if (gamepad2.bWasPressed()) {
            robot.s.addLoopAction(() -> {
                hasBALoop = true;
                robot.s.getLoopAction("loopC").killAction = true;
            }, 1.5, "loopC");
            robot.s.addLoopAction(() -> {
                hasBBLoop = true;
                robot.s.getLoopAction("loopD").killAction = true;
            }, 2, "loopD");
        }

        telemetry.addData("loopXCount", numberXLoop);

        /*
        * Finally, a loop action that doesn't commit suicide!
        * This loop action increments a value forever. This behaves similarly to the
        * action on line 94, but has one noticeable difference. When releasing your
        * finger, it doesn't take the delay time to fully stop - it promptly stops.
        */
        if (gamepad2.xWasPressed()) {
            hasXRun = true;
            robot.s.addLoopAction(() -> {
                numberXLoop++;
            }, 1, "loopE");
        }
        //This kills the loop action if x is no longer pressed.
        if ((!gamepad2.x) && hasXRun) {
            hasXRun = false;
            robot.s.getLoopAction("loopE").killAction = true;
        }

        //loop() is one of the most important components on this system.
        robot.s.loop();

        robot.getOuttake().runOuttakePID(telemetry);
        telemetry.update();
    }
}