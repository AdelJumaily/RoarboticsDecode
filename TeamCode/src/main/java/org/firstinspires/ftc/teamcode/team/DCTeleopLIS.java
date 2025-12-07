package org.firstinspires.ftc.teamcode.team;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team.auto.DCBaseLIS;
import org.firstinspires.ftc.teamcode.team.config.DCRobotConfig;
import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;
import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;


/*
 * This {@code class} acts as the driver-controlled program for FTC team 16598 for the Into The Deep
 * challenge. By extending {@code ITDTeleopRobotCHALLC}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks          -> Mecanum drive
 *          Left-Bumper                     -> Decrease robot speed .7x
 *          Right-Bumper                    -> Normal robot speed 1x
 *      Intake:
 *          Right_trigger                   -> Spins to Intake
 *          Left_trigger                    -> Idle
 *
 *      Shooter:
 *          A-Button                        -> Spins to Shoot
 *          B-Button                        -> Idle
 *
 *      Lift:
 *          Dpad-Down                       -> Extend lift to "Out" position
 *          Dpad-Up                         -> Extend lift to "Out" position
 *
 *
 *  User 2:
 *
 *
 */
@TeleOp(name = "DC TeleOp LIS", group = "Main")
public class DCTeleopLIS extends DCTeleopRobotLIS {

    private double currentTime = 0;
    private double speedMultiplier = DCRobotConfig.DriveSpeed.SLOW;

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

        // Speed control
        if (getEnhancedGamepad1().isLeftBumperJustPressed()) {
            speedMultiplier = DCRobotConfig.DriveSpeed.SLOW;
        }
        if (getEnhancedGamepad1().isRightBumperJustPressed()) {
            speedMultiplier = DCRobotConfig.DriveSpeed.NORMAL;
        }


        // Intake control
        if (getEnhancedGamepad1().getRight_trigger() > 0) {
            drive.robot.getIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.INTAKE);
        }
        if (getEnhancedGamepad1().getLeft_trigger() > 0) {
            drive.robot.getIntakeSubsystem().getStateMachine().updateState(DCIntakeStateMachine.State.IDLE);
        }

        // Shooter control
        if (getEnhancedGamepad1().isAPressed()) {
            drive.robot.getShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.SHOOT);
        }
        if (getEnhancedGamepad1().isBPressed()) {
            drive.robot.getShooterSubsystem().getStateMachine().updateState(DCShooterStateMachine.State.IDLE);
        }

        // Lift control
        if (getEnhancedGamepad1().isDpadDownJustPressed()) {
            drive.robot.getLiftSubsystem().extend(DCRobotConfig.LiftPositions.LIFT_OUT);
        }
        if (getEnhancedGamepad1().isDpadUpJustPressed()) {
            drive.robot.getLiftSubsystem().extend(DCRobotConfig.LiftPositions.LIFT_IN);
        }



//---------------------------------------------------------------------------------------------------------------------------------------------------------

        telemetry.addData("Intake State: ", drive.robot.getIntakeSubsystem().getState());
        telemetry.addData("Shooter State: ", drive.robot.getShooterSubsystem().getState());
        telemetry.addData("Lift State: ", drive.robot.getLiftSubsystem().getState());


        updateTelemetry(telemetry);
        currentTime = getRuntime();
    }

}
