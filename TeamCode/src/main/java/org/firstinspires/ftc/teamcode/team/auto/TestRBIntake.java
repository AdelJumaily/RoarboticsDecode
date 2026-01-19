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


@Autonomous(name = "Red back TEST", group = "Pixel")
public class TestRBIntake extends LinearOpMode { //updated




    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;


    private static final double width = 16.25;
    private static final double length = 16;



    public static double DISTANCE = 5;
    static final Vector2d path0 = new Vector2d(-46,46); //gets ready to shoot artifacts
//    static final Vector2d path1 = new Vector2d(-12,24); //gets ready to intake artifacts
//    static final Vector2d path2 = new Vector2d(-12,60); //intakes artifacts
//    static final Vector2d path3 = new Vector2d(-46,46); //gets ready to shoot artifacts
    static final Vector2d path1 = new Vector2d(-12,50); // parks outside of the zone

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);




    enum State {

        IDLE,
        WAIT0,
        MTSP, //move to shooting position
        Shoot1, //shoots the 3 balls 1st time
        MTBLP, //move to Ball left position
        MTBRP, //move to Ball right position and
        Shoot2, //shoots the 3 balls 2nd time
        End, //


    }


    TestRBIntake.State currentState = State.IDLE;


    Pose2d startPoseRL = new Pose2d(-50.5, 50.25, Math.toRadians(120));
    //lift test needs to be done (values are estimated/inaccurate)




    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));


        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPoseRL);


        TrajectorySequence P0 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path0)
                .turn(Math.toRadians(120)) //-0.610865
                .build();


//        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
//                .lineTo(path1)
//                .build();
//
//
//        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P1.end())
//                .lineTo(path2)
//                .build();
//
//
//        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
//                .lineTo(path3)
//                .build();

       TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
              .lineTo(path1)
               .build();

/*
       TrajectorySequence P5 = drive.trajectorySequenceBuilder(P4.end())
               .lineTo(path5)
               .build();


       TrajectorySequence P6 = drive.trajectorySequenceBuilder(P5.end())
               .lineTo(path6)
               .build();
       */
        drive.getExpansionHubs().update(getDt());
//        drive.robot.getDCIntakeSubsystem().update(getDt());
        //drive.robot.getITDClawStateMachine().update(getDt());




        double t1 = waitTimer.milliseconds();


        double t2 = waitTimer.milliseconds();


        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();


        telemetry.update();
        waitForStart();

//hello

        if (isStopRequested()) return;


        currentState = State.WAIT0;


        while (opModeIsActive() && !isStopRequested()) {

            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));


            switch (currentState) {

                case IDLE:
                    PoseStorage.currentPose = drive.getPoseEstimate();
                    break;

                case WAIT0:
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(P0);
                        currentState = State.Shoot1;
                        waitTimer.reset();
                    }

//                case Shoot1:
//                    drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
//                    waitTimer.reset();
//                    while (drive.robot.getDCShooterSubsystem().getStateMachine().getState() == DCShooterStateMachine.State.SHOOT) {
//                        if (waitTimer.milliseconds() >= 2000){
//                            drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
//                        }
//                    }
//                    if(!drive.isBusy()) {
//                        currentState = State.MTSP;
//                    }
//                    break;
//
//                case MTSP:
//                    if (!drive.isBusy()) {
//                        drive.followTrajectorySequenceAsync(P1);
//                        currentState = State.MTBLP;
//                    }
//                    break;
//
//
//                case MTBLP:
//                    drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
//                    drive.followTrajectorySequenceAsync(P2);
//                    if(!drive.isBusy()){
//                        drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
//                        currentState = State.MTBRP;
//                    }
//                    break;
//
//                case MTBRP:
//                    if(!drive.isBusy()) {
//                        drive.followTrajectorySequenceAsync(P3);
//                        currentState = State.Shoot2;
//                    }
//                    break;
//
//                case Shoot2:
//                    drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
//                    waitTimer.reset();
//                    while (drive.robot.getDCShooterSubsystem().getStateMachine().getState() == DCShooterStateMachine.State.SHOOT) {
//                        if (waitTimer.milliseconds() >= 2000){
//                            drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
//                        }
//                    }
//                    if(!drive.isBusy()) {
//                        currentState = State.End;
//                    }
//                    break;

                case End:
                    if(!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(P1);
                    }
                    break;
            }




            drive.update();


            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            drive.getExpansionHubs().update(getDt());
            //drive.robot.getDCLiftSubsystem().update(getDt());
//            drive.robot.getDCIntakeSubsystem().update(getDt());


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
