package org.firstinspires.ftc.teamcode.team.auto;




import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;




@Autonomous(name = "Blue back", group = "Pixel")
public class BlueBack extends LinearOpMode { //updated








    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;


    private static final double width = 16.25;
    private static final double length = 16;


    static final Vector2d path0 = new Vector2d(-24,-3);
    static final Vector2d path1 = new Vector2d(-12, 54);
    static final Vector2d path2 = new Vector2d(-38,13);
    static final Vector2d path3 = new Vector2d(-58,13);
    static final Vector2d path4 = new Vector2d(-46.5,46.25);
    static final Vector2d path5 = new Vector2d(-12,54);

    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);








    enum State {
        WAIT0,
        MTSP, //move to shooting position
        Shoot1, //shoots the 3 balls 1st time
        MTBLP, //move to Ball left position
        MTBRP, //move to Ball right position and
        Shoot2, //shoots the 3 balls 2nd time
        End, //




    }




    BlueBack.State currentState = BlueBack.State.WAIT0;




    Pose2d startPoseRL = new Pose2d(51.5, 51.25);
    //lift test needs to be done (values are estimated/inaccurate)








    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));




        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPoseRL);
        drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);




        TrajectorySequence P4 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path4)
                .turn(35)
                .build();




        TrajectorySequence P5 = drive.trajectorySequenceBuilder(P4.end())
                .lineTo(path5)
                .build();




        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P5.end())
                .lineTo(path2)
                .build();




        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
                .lineTo(path3)
                .build();




        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P3.end())
                .lineTo(path1)
                .build();




        TrajectorySequence P0 = drive.trajectorySequenceBuilder(P1.end())
                .lineTo(path0)
                .build();




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
        drive.robot.getDCShooterSubsystem().update(getDt());
        //drive.robot.getITDClawStateMachine().update(getDt());








        double t1 = waitTimer.milliseconds();




        double t2 = waitTimer.milliseconds();




        telemetry.addData("Initialize Time Seconds", (t2 - t1));
        telemetry.update();




        telemetry.update();
        waitForStart();




        if (isStopRequested()) return;




        currentState = BlueBack.State.WAIT0;




        while (opModeIsActive() && !isStopRequested()) {




            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));




            switch (currentState) {




                case WAIT0:
                    if (waitTimer.milliseconds() >= 1000)
                        currentState = State.MTSP;
                    waitTimer.reset();
                    telemetry.addLine("in the wait0 state");
                    break;




                case MTSP:
                    drive.followTrajectorySequenceAsync(P1);
                    if (!drive.isBusy()) {
                        currentState = State.Shoot1;
                    }




                case Shoot1:
                    drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
                    waitTimer.reset();
                    while (drive.robot.getDCShooterSubsystem().getStateMachine().getState() == DCShooterStateMachine.State.SHOOT) {
                        if (waitTimer.milliseconds() >= 2000){
                            drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
                        }
                    }
                    if(!drive.isBusy()) {
                        currentState = State.MTBLP;
                    }




                case MTBLP:
                    drive.followTrajectorySequenceAsync(P2);
                    if(!drive.isBusy()){
                        currentState = State.MTBRP;
                    }


                case MTBRP:
                    drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
                    drive.followTrajectorySequenceAsync(P3);
                    if(!drive.isBusy()) {
                        drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
                        drive.followTrajectorySequenceAsync(P1);
                    }
                case Shoot2:
                    drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
                    waitTimer.reset();
                    while (drive.robot.getDCShooterSubsystem().getStateMachine().getState() == DCShooterStateMachine.State.SHOOT) {
                        if (waitTimer.milliseconds() >= 2000){
                            drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
                        }
                    }
                    if(!drive.isBusy()) {
                        currentState = State.End;
                    }




                case End:
                    drive.followTrajectorySequenceAsync(P0);
                    if(!drive.isBusy()) {
                        break;
                    }




            }


            drive.update();




            //The following code ensure state machine updates i.e. parallel execution with drivetrain
            //drive.getDCExpansionHubsLIS().update(getDt());
            //drive.robot.getDCLiftSubsystem().update(getDt());
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





