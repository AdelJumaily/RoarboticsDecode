package org.firstinspires.ftc.teamcode.team.auto;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.DCAgitatorStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;


@Autonomous(name = "Intake test", group = "Pixel")



public class Intake_Test extends LinearOpMode { //updated

    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;


    private static final double width = 18;
    private static final double length = 18;



    static final Vector2d path5 = new Vector2d(24,4.4); // Shooting position
    static final Vector2d path0 = new Vector2d(-24,24); // Shooting position
    static final Vector2d path1 = new Vector2d(-12, 24); // Move to pick up balls part 1
    static final Vector2d path2 = new Vector2d(-12,32); // Move to pick up balls part 2 (optional)
    static final Vector2d path3 = new Vector2d(-12,50); // Move to pick up balls part 3
    static final Vector2d path4 = new Vector2d(-24,24);  // Shooting position
    static final Vector2d path6 = new Vector2d(20,20); // Shooting position


    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);




    // This defines all the states the robot can be in during auto
    enum State {
        WAIT0,
        MTSP, // Moves to shooting position
         MTBLP, // Moves the ball to left position
        MTBRP, // Moves the ball to right position
        END
    }


    Intake_Test.State currentState = Intake_Test.State.WAIT0;


    Pose2d startPoseRL = new Pose2d (-50.5, -50.25);
    //lift test needs to be done (values are estimated/inaccurate)




    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));


        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPoseRL);
        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.IDLE);
        // tells the robot where it could go does not actually move the robot
        TrajectorySequence P5 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path5)
                .turn(Math.toRadians(-30))
                .build();


        TrajectorySequence P0 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path0)
                .turn(Math.toRadians(225)) // turns toward the baskets
                .build();


        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
                .turn(Math.toRadians(45))
                .lineTo(path1)
                .turn(Math.toRadians(90))
                .build();


        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P0.end())
                .lineTo(path2)
                .build();


        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
                .lineTo(path3)
                .build();


        TrajectorySequence P4 = drive.trajectorySequenceBuilder(P3.end())
                .turn(Math.toRadians(155))
                .lineTo(path4)
                .turn(Math.toRadians(65))
                .build();


        TrajectorySequence P6 = drive.trajectorySequenceBuilder(P4.end())
                .lineTo(path6)
                .build();


        //drive.getITDExpansionHubsLACH().update(getDt());
        drive.robot.getDCintakeSubsystem().update(getDt());
        //drive.robot.getITDClawStateMachine().update(getDt());




        double t1 = waitTimer.milliseconds();


        double t2 = waitTimer.milliseconds();


        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();


        telemetry.update();
        waitForStart();


        if (isStopRequested()) return;


        currentState = State.WAIT0;


        while (opModeIsActive() && !isStopRequested()) {


            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));


            switch (currentState) {


                case WAIT0:
                    if (waitTimer.milliseconds() >= 2000) {
                        drive.followTrajectorySequenceAsync(P0);
                        currentState = State.MTSP;
                        waitTimer.reset();
                    }
                    telemetry.addLine("in the wait0 state");
                    break;


                case MTSP:
                    if (!drive.isBusy()) {
                        currentState = State.MTBLP;
                        waitTimer.reset();
                    }
                    break;

                case MTBLP:
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(P2);
                        while(drive.isBusy()) {
                            telemetry.addLine("moving to balls");
                        }
                        currentState = State.MTBRP;
                        drive.followTrajectorySequenceAsync(P3);
                    }
                    break;

                case MTBRP:
                    drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
                    drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.Ajadate);
                    if(!drive.isBusy()) {
                        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.IDLE);
                        drive.followTrajectorySequenceAsync(P4);
                        currentState = State.END;
                    }
                    break;

                case END:
                    break;


            }

            drive.update();


            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            //drive.getDCExpansionHubsLIS().update(getDt());
            drive.robot.getDCLiftSubsystem().update(getDt());
            drive.robot.getDCintakeSubsystem().update(getDt());
            drive.robot.getDCShooterSubsystem().update(getDt());


            telemetry.update();
        }


        drive.setMotorPowers(0.0,0.0,0.0,0.0);
    }
    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }


    public static void setUpdateRuntime(TimeProfiler updaRuntime) {
        updateRuntime = updaRuntime;
    }


    public static double getDt() {
        return dt;
    }


    public static void setDt(double pdt) {
        dt = pdt;
    }
}

