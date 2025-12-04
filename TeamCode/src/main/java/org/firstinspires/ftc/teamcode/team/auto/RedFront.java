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




@Autonomous(name = "Red front", group = "Pixel")

public class RedFront extends LinearOpMode { //updated




    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;


    private static final double width = 16.25;
    private static final double length = 16;






    static final Vector2d path0 = new Vector2d(36 ,0); // MTSP
    static final Vector2d path1 = new Vector2d(55.7, -55.5); // Moves to Ball left position
    static final Vector2d path2 = new Vector2d(63.9,-52); //Moves to Ball right position




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


    RedFront.State currentState = RedFront.State.WAIT0;


    Pose2d startPoseRL = new Pose2d (64.5, -32.1);
    //lift test needs to be done (values are estimated/inaccurate)




    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));


        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPoseRL);
        drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
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


        currentState = RedFront.State.WAIT0;


        while (opModeIsActive() && !isStopRequested()) {


            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));


            switch (currentState) {


                case WAIT0:
                    telemetry.addLine("in the wait0 state");
                    waitTimer.reset();
                    if (waitTimer.milliseconds() >= 2000) { // 2 seconds passed
                        currentState = State.MTSP;
                    }
                    break;

                case MTSP:
                    drive.followTrajectorySequenceAsync(P0);                    if (!drive.isBusy()) {
                        telemetry.addLine("in the MTSP state");
                        currentState = State.SHOOT1;
                    }
                    break;

                case SHOOT1:
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
                    drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
                    drive.followTrajectorySequenceAsync(P1);
                    if(!drive.isBusy()) {
                        drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
                        drive.followTrajectorySequenceAsync(P2);
                        currentState = State.MTBRP;
                    }

                case MTBRP:
                    drive.followTrajectorySequenceAsync(P0);
                    if(!drive.isBusy()) {
                        currentState = State.SHOOT2;
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
                    drive.followTrajectorySequenceAsync(P1);
                    if(!drive.isBusy()) {
                        break;
                    }
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

