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
import org.firstinspires.ftc.teamcode.team.states.DCServoStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;


@Autonomous(name = "CSHS codeAns", group = "Pixel")
public class CSHS_codeAns extends LinearOpMode {






    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;




    private static final double width = 18;
    private static final double length = 18;






    public static double DISTANCE = 5;

//—-------------------------------------------------------------------------
//Create the paths the robot goes to


    static final Vector2d path0 = new Vector2d(60,12);
    static final Vector2d path1 = new Vector2d(60,60);
    static final Vector2d path2 = new Vector2d(5,60);
    static final Vector2d path3 = new Vector2d(60,108);

















//—-------------------------------------------------------------------------








    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);





//—-------------------------------------------------------------------------
//Define cases here


    enum State {
        Start,
        Intake,
        Shoot,
        End






    }

//—-------------------------------------------------------------------------








    Pose2d startPoseRL = new Pose2d(12, 12, Math.toRadians(0));








    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));
        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPoseRL);

//—-------------------------------------------------------------------------
//Create the Trajectory Sequence Path Builder


        TrajectorySequence P1 = drive.trajectorySequenceBuilder(startPoseRL)
                .lineTo(path0)
                .lineTo(path1)
                .turn(Math.toRadians(90))
                .build();

        TrajectorySequence P2 =  drive.trajectorySequenceBuilder(P1.end())
                .lineTo(path2)
                .turn(Math.toRadians(-180))
                .lineTo(path3)
                .build();

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

        State currentState = State.Start;


        while (opModeIsActive() && !isStopRequested()) {


            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

//—-------------------------------------------------------------------------
            //create the cases
            switch (currentState) {

                case Start:
                    while (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(P1);
                        currentState = State.Intake;
                    }

                    break;

                case Intake:
                    while (!drive.isBusy()) {
                        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.Ajadate);
                        drive.followTrajectorySequenceAsync(P2);
                        currentState = State.Shoot;
                    }
                    break;

                case Shoot:
                    while (!drive.isBusy()) {
                        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
                        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
                        drive.robot.getDCServoSubsystem().getStateMachine().updateState(DCServoStateMachine.State.Up);
                    }
                    break;
                case End:
                    while (!drive.isBusy()) {
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.IDLE);
                        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
                        drive.robot.getDCServoSubsystem().getStateMachine().updateState(DCServoStateMachine.State.Down);
                    }
                    break;
//—-------------------------------------------------------------------------

            }
                    drive.update();
                    drive.getExpansionHubs().update(getDt());
                    telemetry.update();



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

