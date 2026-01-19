//package org.firstinspires.ftc.teamcode.team;
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.team.auto.DCBaseLIS;
//import org.firstinspires.ftc.teamcode.team.states.DCAgitatorStateMachine;
//import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;
//import org.firstinspires.ftc.teamcode.team.states.DCServoStateMachine;
//import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;
//
//
///*
// * This {@code class} acts as the driver-controlled program for FTC team 16598 for the Into The Deep
// * challenge. By extending {@code ITDTeleopRobotCHALLC}, we already have access to all the robot subsystems,
// * so only tele-operated controls need to be defined here.
// *
// * The controls for this robot are:
// *  User 1:
// *      Drive:
// *          Left & Right joysticks          -> Mecanum drive
// *          Left-Bumper                     -> Decrease robot speed .7x
// *          Right-Bumper                    -> Normal robot speed 1x
// *      Intake:
// *          Right_trigger                   -> Spins to Intake
// *          Left_trigger                    -> Idle
// *
// *      Shooter:
// *          A-Button                        -> Spins to Shoot
// *          B-Button                        -> Idle
// *
// *      Lift:
// *          Dpad-Down                       -> Extend lift to "Out" position
// *          Dpad-Up                         -> Extend lift to "Out" position
// *
// *
// *  User 2:
// *
// *
// */
//@TeleOp(name = "DC TeleOp Servo", group = "Main")
//public class DCTeleopServo_Test extends DCTeleopRobotLIS {
//
//    private double currentTime = 0; // keep track of current time
//    private double speedMultiplier = 0.7;
//    private double LiftOut = 0d;
//    private double LiftIn = 10d;
//
//    private Pose2d poseEstimate;
//
//    @Override
//    public void init(){
//        drive = new DCBaseLIS(hardwareMap, true);
//        super.init();
//    }
//
//    @Override
//    public void start() {
//    }
//
//    @Override
//    public void loop() {
//        super.loop();
//        drive.update();
//        poseEstimate = drive.getPoseEstimate();
//
//        //---------------------------------------------------------------------------------------------------------------------------------------------------------
//        //Gamepad 1
//
//
//        drive.setWeightedDrivePower(
//                new Pose2d(
//                        -gamepad1.left_stick_y * speedMultiplier,
//                        -gamepad1.left_stick_x * speedMultiplier,
//                        -gamepad1.right_stick_x * speedMultiplier
//                )
//        );
//
//
//        //Servo
//        telemetry.addData("Servo State: ", drive.robot.getDCServoSubsystem().getStateMachine().getState());
//        if (getEnhancedGamepad1().getRight_trigger() > 0) {
//            drive.robot.getDCServoSubsystem().getStateMachine().updateState(DCServoStateMachine.State.Up);
//        }
//        if (getEnhancedGamepad1().getLeft_trigger() > 0) {
//            drive.robot.getDCServoSubsystem().getStateMachine().updateState(DCServoStateMachine.State.Down);
//        }
//
//
////        //Lift
////        if(getEnhancedGamepad1().isDpadDownJustPressed()){
////            drive.robot.getDCLiftSubsystem().extend(LiftOut);
////        }
////        if(getEnhancedGamepad1().isDpadUpJustPressed()){
////            drive.robot.getDCLiftSubsystem().extend(LiftIn);
////        }
//
//
//
////---------------------------------------------------------------------------------------------------------------------------------------------------------
//
//        telemetry.addData("Intake State: ", drive.robot.getDCintakeSubsystem().getStateMachine().getState());
//        telemetry.addData("Shooter State: ", drive.robot.getDCShooterSubsystem().getStateMachine().getState());
////        telemetry.addData("Lift State: ", drive.robot.getDCLiftSubsystem().getStateMachine().getState());
//        telemetry.addData("Ajadator State: ", drive.robot.getDCajadatorSubsystem().getStateMachine().getState());
//        telemetry.addData("Servo State: ", drive.robot.getDCServoSubsystem().getStateMachine().getState());
//
//
//
//        updateTelemetry(telemetry);
//        currentTime = getRuntime();
//    }
//
//}
