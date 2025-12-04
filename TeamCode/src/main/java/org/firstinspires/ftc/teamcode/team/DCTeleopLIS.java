package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.auto.DCBaseLIS;
import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;
import org.firstinspires.ftc.teamcode.team.states.IntakeStateMachine;


/*
 * This {@code class} acts as the driver-controlled program for FTC team 16598 for the Into The Deep
 * challenge. By extending {@code ITDTeleopRobotCHALLC}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks      -> Mecanum drive
 *          Left-Bumper                 -> Decrease robot speed .7x
 *          Right-Bumper                -> Normal robot speed 1x
 *      Intake:
 *          Left_trigger                -> Spins to Intake
 *          Right_trigger               -> Spins to Outtake
 *
 *      Shooter:
 *          Left_trigger                -> Spins to Intake
 *          Right_trigger               -> Spins to Outtake
 *      Lift:
 *          Y-Button                    -> Extend lift to "Out" position
 *          B-Button                    -> Extend lift to "In" position
 *          A-Button                    -> Retract lift to starting position
 *
 *
 *  User 2:
 *
 *
 */
@TeleOp(name = "DC TeleOp LIS", group = "Main")
public class DCTeleopLIS extends DCTeleopRobotLIS {

    private double currentTime = 0; // keep track of current time
    private double speedMultiplier = 0.7;
    private double LiftOut = 0d;
    private double LiftIn = 10d;

    private Pose2d poseEstimate;

    @Override
    public void init(){
        drive = new DCBaseLIS(hardwareMap, true);
        super.init();
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        super.loop();
        drive.update();
        poseEstimate = drive.getPoseEstimate();

        //---------------------------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 1


        drive.setWeightedDrivePower(
                new Pose2d(
                        -gamepad1.left_stick_y * speedMultiplier,
                        -gamepad1.left_stick_x * speedMultiplier,
                        -gamepad1.right_stick_x * speedMultiplier
                )
        );

        //This changes the speed the robot moves at
        if (getEnhancedGamepad1().isLeftBumperJustPressed()) {
            speedMultiplier = 0.7;
        }
        if (getEnhancedGamepad1().isRightBumperJustPressed()) {
            speedMultiplier = 1.0;
        }


        //Intake
        //spins the intake to intake a pixel
        if (getEnhancedGamepad1().getLeft_trigger() > 0) {
            drive.robot.getDCIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
        }
        if(getEnhancedGamepad1().isDpadUpJustPressed()){
            drive.robot.getDCLiftSubsystem().extend(LiftIn);
        }


        //Lift
        if(getEnhancedGamepad1().isDpadDownJustPressed()){
            drive.robot.getDCLiftSubsystem().extend(LiftOut);
        }
        if(getEnhancedGamepad1().isDpadUpJustPressed()){
            drive.robot.getDCLiftSubsystem().extend(LiftIn);
        }



//---------------------------------------------------------------------------------------------------------------------------------------------------------


        telemetry.addData("Intake State: ", drive.robot.getDCIntakeSubsystem().getStateMachine().getState());
        telemetry.addData("Shooter State: ", drive.robot.getDCShooterSubsystem().getStateMachine().getState());
        telemetry.addData("Lift State: ", drive.robot.getDCLiftSubsystem().getStateMachine().getState());


        updateTelemetry(telemetry);
        currentTime = getRuntime();
    }

}
