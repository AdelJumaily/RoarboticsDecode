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




@Autonomous(name = "Blue front", group = "Pixel")

public class BlueFront extends LinearOpMode { //updated




    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;


    private static final double width = 16.25;
    private static final double length = 16;



    static final Vector2d path0 = new Vector2d(-24,48);
    static final Vector2d path1 = new Vector2d(-36, 48);
    static final Vector2d path2 = new Vector2d(-12,-12);
    static final Vector2d path3 = new Vector2d(-60,12);
    static final Vector2d path4 = new Vector2d(-36,-12);





    //ElapsedTime carouselTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);




    // This defines all the states the robot can be in during auto
    enum State {
        WAIT0,
        MTSP, // Moves to shooting position
        SHOOT1, // Shoots 3 balls
        MTBLP, // Moves the ball to left position
        MTBRP, // Moves the ball to right position
        SHOOT2, // Shoots 3 balls
        END
    }


    BlueFront.State currentState = BlueFront.State.WAIT0;


    Pose2d startPoseRL = new Pose2d (-24, -72);
    //lift test needs to be done (values are estimated/inaccurate)




    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));


        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPoseRL);
        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
        // tells the robot where it could go does not actually move the robot
        TrajectorySequence P0 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path0) // MTSP
                .build();


        TrajectorySequence P1 = drive.trajectorySequenceBuilder(P0.end())
                .lineTo(path1)
                .build();


        TrajectorySequence P2 = drive.trajectorySequenceBuilder(P1.end())
                .lineTo(path2)
                .build();

        TrajectorySequence P3 = drive.trajectorySequenceBuilder(P2.end())
                .lineTo(path3)
                .build();

        TrajectorySequence P4 = drive.trajectorySequenceBuilder(P3.end())
                .lineTo(path4)
                .turn(35)
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
                    if (waitTimer.milliseconds() >= 1000)
                        currentState = BlueFront.State.MTSP;
                    waitTimer.reset();
                    telemetry.addLine("in the wait0 state");
                    break;


                case MTSP:
                    drive.followTrajectorySequenceAsync(P0);
                    if (waitTimer.milliseconds() >= 500) {
                        drive.followTrajectorySequenceAsync(P1);
                    }
                    if (!drive.isBusy()) {
                        currentState = BlueFront.State.SHOOT1;
                    }


                case SHOOT1:
                    drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
                    waitTimer.reset();
                    while (drive.robot.getDCShooterSubsystem().getStateMachine().getState() == DCShooterStateMachine.State.SHOOT) {
                        if (waitTimer.milliseconds() >= 2000){
                            drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
                            waitTimer.reset();
                        }
                    }
                    if(!drive.isBusy()) {
                        currentState = BlueFront.State.MTBLP;
                    }






                case MTBLP:
                    drive.followTrajectorySequenceAsync(P2);
                    if (waitTimer.milliseconds() >= 200) {
                        drive.followTrajectorySequenceAsync(P3);
                    }
                    if(!drive.isBusy()){
                        currentState = State.MTBRP;
                    }

                case MTBRP:
                    drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
                    drive.followTrajectorySequenceAsync(P1);
                    if(!drive.isBusy()) {
                        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
                        drive.followTrajectorySequenceAsync(P2);
                    }
                case SHOOT2:
                    drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
                    waitTimer.reset();
                    while (drive.robot.getDCShooterSubsystem().getStateMachine().getState() == DCShooterStateMachine.State.SHOOT) {
                        if (waitTimer.milliseconds() >= 2000){
                            drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
                        }
                    }
                    if(!drive.isBusy()) {
                        currentState = State.END;
                    }


                case END:
                    drive.followTrajectorySequenceAsync(P4);
                    if(!drive.isBusy()) {
                        break;
                    }


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

