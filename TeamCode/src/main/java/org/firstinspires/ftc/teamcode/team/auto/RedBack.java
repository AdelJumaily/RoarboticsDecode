package org.firstinspires.ftc.teamcode.team.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.PoseStorage;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;


@Autonomous(name = "Red back", group = "Pixel")
public class RedBack extends LinearOpMode { //updated


    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    private static final double width = 16.375;
    private static final double length = 15.125;



    static final Vector2d path0 = new Vector2d(36 ,0);
    static final Vector2d path1 = new Vector2d(55.7, -55.5);
    static final Vector2d path2 = new Vector2d(63.9,-52); //observation zone


    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    enum State {
        WAIT0,
    }

    RedBack.State currentState = RedBack.State.WAIT0;

    Pose2d startPoseRL = new Pose2d(64.5, -32.1);
    //lift test needs to be done (values are estimated/inaccurate)


    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPoseRL);
        drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);

        TrajectorySequence P0 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path0)
                .build();

        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
                .lineTo(path1)
                .build();

        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P1.end())
                .lineTo(path2)
                .build();

//        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
//                .lineTo(path3)
//                .build();
/*
        TrajectorySequence P4 = drive.trajectorySequenceBuilder(P3.end())
                .lineTo(path4)
                .build();

        TrajectorySequence P5 = drive.trajectorySequenceBuilder(P4.end())
                .lineTo(path5)
                .build();

        TrajectorySequence P6 = drive.trajectorySequenceBuilder(P5.end())
                .lineTo(path6)
                .build();
        */
        //drive.getITDExpansionHubsLACH().update(getDt());
        drive.robot.getDCIntakeSubsystem().update(getDt());
        //drive.robot.getITDClawStateMachine().update(getDt());


        double t1 = waitTimer.milliseconds();

        double t2 = waitTimer.milliseconds();

        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();

        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        currentState = RedBack.State.WAIT0;

        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {

                case WAIT0:
                    telemetry.addLine("in the wait0 state");
                    break;


            }

            drive.update();

            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            //drive.getDCExpansionHubsLIS().update(getDt());
            drive.robot.getDCLiftSubsystem().update(getDt());
            drive.robot.getDCIntakeSubsystem().update(getDt());
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