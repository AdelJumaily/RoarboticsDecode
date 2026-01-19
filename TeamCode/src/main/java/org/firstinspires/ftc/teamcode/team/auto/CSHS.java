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
import org.firstinspires.ftc.teamcode.team.states.DCAgitatorStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCServoStateMachine;



@Autonomous(name = "CSHS", group = "Pixel")
public class CSHS extends LinearOpMode { //updated

    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    private static final double width = 18;
    private static final double length = 18;

    public static double DISTANCE = 5;


//—-------------------------------------------------------------------------
//Create the paths the robot goes to

    static final Vector2d path0 = new Vector2d(67,67);



//—-------------------------------------------------------------------------

    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

//—-------------------------------------------------------------------------
//Define cases here


    enum State {
        Example,





    }

//—-------------------------------------------------------------------------

    Pose2d startPoseRL = new Pose2d(0, 0, Math.toRadians(0));



    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));
        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPoseRL);


//—-------------------------------------------------------------------------
//Create the Trajectory Sequence Path Builder




        //     TrajectorySequence Example = drive.trajectorySequenceBuilder(startPoseRL)
        //                .lineTo(Example)
        //             .turn(Math.toRadians(120))
        //             .build();




//—-------------------------------------------------------------------------




        drive.getExpansionHubs().update(getDt());
        drive.robot.getDCintakeSubsystem().update(getDt());
        drive.robot.getDCShooterSubsystem().update(getDt());
        drive.robot.getDCAgitatorSubsystem().update(getDt());
        drive.robot.getDCServoSubsystem().update(getDt());




        double t1 = waitTimer.milliseconds();




        double t2 = waitTimer.milliseconds();




        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();




        telemetry.update();
        waitForStart();


        if (isStopRequested()) return;


        State currentState = State.Example;




        while (opModeIsActive() && !isStopRequested()) {




            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));


//—-------------------------------------------------------------------------
            //create the cases
            switch (currentState) {


                case Example:












//—-------------------------------------------------------------------------




                    drive.update();
                    drive.getExpansionHubs().update(getDt());
                    telemetry.update();
            }




            drive.setMotorPowers(0.0, 0.0, 0.0, 0.0);
        }
    }
    public static TimeProfiler getUpdateRuntime () {
        return updateRuntime;
    }




    public static void setUpdateRuntime (TimeProfiler updaRuntime){
        updateRuntime = updaRuntime;
    }




    public static double getDt () {
        return dt;
    }




    public static void setDt ( double pdt){
        dt = pdt;
    }
}