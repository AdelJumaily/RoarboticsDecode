package org.firstinspires.ftc.teamcode.team.auto.red;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team.auto.DCBaseLIS;
import org.firstinspires.ftc.teamcode.team.odometry.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCAgitatorStateMachine;

@Autonomous(name = "Red Back", group = "Pixel")
public class RedBack extends LinearOpMode {

    DCBaseLIS drive;
    private static double dt;
    private static TimeProfiler updateRuntime;

    ElapsedTime waitTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    // Position coordinates - TODO: Update these with actual position coordinates
    // Position 1
    static final Vector2d position1 = new Vector2d(0, 0); // TODO: Put in the position coordinates
    
    // Position 2
    static final Vector2d position2 = new Vector2d(0, 0); // TODO: Put in the position coordinates
    
    // Position 3
    static final Vector2d position3 = new Vector2d(0, 0); // TODO: Put in the position coordinates
    
    // Position 4
    static final Vector2d position4 = new Vector2d(0, 0); // TODO: Put in the position coordinates

    enum State {
        CASE1_WAIT,      // Wait 2000 milliseconds
        CASE2_POSITION1, // Go to 1st Position, Turn ON Shooter, Wait 500ms, Turn ON Agitator
        CASE3_WAIT_OFF,  // Wait 1000ms, Turn OFF Shooter, Turn OFF Agitator
        CASE4_POSITION2, // Go to 2nd Position, Turn ON Agitator, Turn ON Intake
        CASE5_POSITION3, // Go to 3rd Position, Turn OFF Agitator, Turn OFF Intake
        CASE6_POSITION4, // Go to 4th Position, Turn ON Shooter, Wait 500ms, Turn ON Agitator
        CASE7_FINAL,     // Wait 1000ms, Turn OFF Shooter, Turn OFF Agitator, Turn OFF Intake, Go to 4th Position
        END
    }

    RedBack.State currentState = RedBack.State.CASE1_WAIT;
    private boolean trajectoryStarted = false;

    Pose2d startPose = new Pose2d(-24, -72, 0); // Adjust starting position as needed

    @Override
    public void runOpMode() throws InterruptedException {
        setUpdateRuntime(new TimeProfiler(false));

        drive = new DCBaseLIS(hardwareMap);
        drive.setPoseEstimate(startPose);
        
        // Initialize subsystems to IDLE
        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.IDLE);

        // Build trajectory sequences
        TrajectorySequence toPosition1 = drive.trajectorySequenceBuilder(startPose)
                .lineTo(position1)
                .build();

        TrajectorySequence toPosition2 = drive.trajectorySequenceBuilder(toPosition1.end())
                .lineTo(position2)
                .build();

        TrajectorySequence toPosition3 = drive.trajectorySequenceBuilder(toPosition2.end())
                .lineTo(position3)
                .build();

        TrajectorySequence toPosition4 = drive.trajectorySequenceBuilder(toPosition3.end())
                .lineTo(position4)
                .build();

        TrajectorySequence backToPosition4 = drive.trajectorySequenceBuilder(toPosition4.end())
                .lineTo(position4)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        currentState = State.CASE1_WAIT;
        waitTimer.reset();

        while (opModeIsActive() && !isStopRequested()) {
            setDt(getUpdateRuntime().getDeltaTime(TimeUnits.SECONDS, true));

            switch (currentState) {
                case CASE1_WAIT:
                    // Case 1: Wait 2000 milliseconds
                    if (waitTimer.milliseconds() >= 2000) {
                        waitTimer.reset();
                        currentState = State.CASE2_POSITION1;
                    }
                    break;

                case CASE2_POSITION1:
                    // Case 2: Go to 1st Position, Check if chassis is not busy, Turn ON Shooter, Wait 500ms, Turn ON Agitator
                    if (!trajectoryStarted) {
                        drive.followTrajectorySequenceAsync(toPosition1);
                        trajectoryStarted = true;
                    }
                    if (!drive.isBusy() && trajectoryStarted) {
                        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
                        waitTimer.reset();
                        trajectoryStarted = false;
                    }
                    if (waitTimer.milliseconds() >= 500 && !drive.isBusy()) {
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.Ajadate);
                        waitTimer.reset();
                        currentState = State.CASE3_WAIT_OFF;
                    }
                    break;

                case CASE3_WAIT_OFF:
                    // Case 3: Wait 1000ms, Turn OFF Shooter, Turn OFF Agitator
                    if (waitTimer.milliseconds() >= 1000) {
                        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.IDLE);
                        waitTimer.reset();
                        currentState = State.CASE4_POSITION2;
                    }
                    break;

                case CASE4_POSITION2:
                    // Case 4: Go to 2nd Position, Check if chassis is not busy, Turn ON Agitator, Turn ON Intake
                    if (!trajectoryStarted) {
                        drive.followTrajectorySequenceAsync(toPosition2);
                        trajectoryStarted = true;
                    }
                    if (!drive.isBusy() && trajectoryStarted) {
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.Ajadate);
                        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
                        trajectoryStarted = false;
                        currentState = State.CASE5_POSITION3;
                    }
                    break;

                case CASE5_POSITION3:
                    // Case 5: Go to 3rd Position, Check if chassis is not busy, Turn OFF Agitator, Turn OFF Intake
                    if (!trajectoryStarted) {
                        drive.followTrajectorySequenceAsync(toPosition3);
                        trajectoryStarted = true;
                    }
                    if (!drive.isBusy() && trajectoryStarted) {
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.IDLE);
                        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
                        trajectoryStarted = false;
                        currentState = State.CASE6_POSITION4;
                    }
                    break;

                case CASE6_POSITION4:
                    // Case 6: Go to 4th Position, Check if chassis is not busy, Turn ON Shooter, Wait 500ms, Turn ON Agitator
                    if (!trajectoryStarted) {
                        drive.followTrajectorySequenceAsync(toPosition4);
                        trajectoryStarted = true;
                    }
                    if (!drive.isBusy() && trajectoryStarted && waitTimer.milliseconds() == 0) {
                        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
                        waitTimer.reset();
                        trajectoryStarted = false;
                    }
                    if (waitTimer.milliseconds() >= 500 && !drive.isBusy()) {
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.Ajadate);
                        waitTimer.reset();
                        currentState = State.CASE7_FINAL;
                    }
                    break;

                case CASE7_FINAL:
                    // Case 7: Wait 1000ms, Turn OFF Shooter, Turn OFF Agitator, Turn OFF Intake, Go to 4th Position
                    if (waitTimer.milliseconds() >= 1000) {
                        drive.robot.getDCShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
                        drive.robot.getDCAgitatorSubsystem().getStateMachine().updateState(DCAgitatorStateMachine.State.IDLE);
                        drive.robot.getDCintakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
                        if (!trajectoryStarted && !drive.isBusy()) {
                            drive.followTrajectorySequenceAsync(backToPosition4);
                            trajectoryStarted = true;
                        }
                        if (!drive.isBusy() && trajectoryStarted) {
                            waitTimer.reset();
                            trajectoryStarted = false;
                            currentState = State.END;
                        }
                    }
                    break;

                case END:
                    // End state - do nothing
                    break;
            }

            // Update drive and subsystems
            drive.update();
            drive.robot.getDCLiftSubsystem().update(getDt());
            drive.robot.getDCintakeSubsystem().update(getDt());
            drive.robot.getDCShooterSubsystem().update(getDt());
            drive.robot.getDCAgitatorSubsystem().update(getDt());
            drive.robot.getDCServoSubsystem().update(getDt());

            telemetry.addData("State", currentState);
            telemetry.addData("Chassis Busy", drive.isBusy());
            telemetry.update();
        }

        drive.setMotorPowers(0.0, 0.0, 0.0, 0.0);
    }

    public static TimeProfiler getUpdateRuntime() {
        return updateRuntime;
    }

    public static void setUpdateRuntime(TimeProfiler updateRuntime) {
        RedBack.updateRuntime = updateRuntime;
    }

    public static double getDt() {
        return dt;
    }

    public static void setDt(double pdt) {
        dt = pdt;
    }
}
