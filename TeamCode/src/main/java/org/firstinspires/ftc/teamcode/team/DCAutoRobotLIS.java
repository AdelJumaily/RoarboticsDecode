package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubMotor;
import org.firstinspires.ftc.teamcode.revextension2.ExpansionHubServo;
import org.firstinspires.ftc.teamcode.team.subsystems.Drive;
import org.firstinspires.ftc.teamcode.team.subsystems.DCExpansionHubsLIS;
import org.firstinspires.ftc.teamcode.team.subsystems.DCLiftSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.DCShooterSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.DCAgitatorSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.DCIntakeSubsystem;
import org.firstinspires.ftc.teamcode.team.subsystems.DCServoSubsystem;


/**
 * Motor naming convention:
 *     Drivetrain
 *         Front Left Wheel -> LF
 *         Back Left Wheel   -> LR
 *         Front Right Wheel -> RF
 *         Back Right Wheel  -> RR
 *      Elevator
 *         Left Motor -> Elev Left
 *         Right Motor -> Elev Right
 *     Arm
 *         Arm Servo  -> Arm
 *      Claw
 *          Gripper -> Claw
 * Misc. sensors naming convention:

 */
public class DCAutoRobotLIS {
    private TimeProfiler matchRuntime;
    private RobotStateEstimator robotStateEstimator;
    private Drive drive;
    private DCExpansionHubsLIS dcExpansionHubsLIS;
    private DCLiftSubsystem DCliftSubsystem;
    private DCShooterSubsystem DCshooterSubsystem;
    private DCAgitatorSubsystem DCagitatorSubsystem;
    private DCServoSubsystem DCservoSubsystem;

    private DCIntakeSubsystem DCintakeSubsystem;


    private RevMotor[] motors;
    private RevServo[] servos;


    public void init(HardwareMap hardwareMap) {
//        setExpansionHubs(new ExpansionHubs(this,
//                hardwareMap.get(ExpansionHubEx.class, "Control Hub"),
//                hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"))
//        );

        setMotors(new RevMotor[] {
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Lift")), false, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 1.503937), //38.2mm diameter
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Intake")), false, false, false, true, Motor.GOBILDA_435_RPM.getENCODER_TICKS_PER_REVOLUTION()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("agitator")), false, false, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION()),
                new RevMotor((ExpansionHubMotor)(hardwareMap.get("Shooter")), false, true, false, true, Motor.GOBILDA_312_RPM.getENCODER_TICKS_PER_REVOLUTION(), 1.503937)
        });

        setServos(new RevServo[] {
                new RevServo((ExpansionHubServo)(hardwareMap.get("Lifter"))),
        });

        setDCLiftSubsystem(new DCLiftSubsystem(getMotors()[0]));
        setDCintakeSubsystem(new DCIntakeSubsystem(getMotors()[1]));
        setDCShooterSubsystem(new DCShooterSubsystem(getMotors()[2]));
        setDCAgitatorSubsystem(new DCAgitatorSubsystem(getMotors()[3]));
        setDCServoSubsystem(new DCServoSubsystem(getServos()[0]));
        setMatchRuntime(new TimeProfiler(false));
    }


    public RevMotor[] getMotors() {
        return motors;
    }

    public void setMotors(RevMotor[] motors) {
        this.motors = motors;
    }

    public RevServo[] getServos() {
        return servos;
    }

    public void setServos(RevServo[] servos) {
        this.servos = servos;
    }

    public DCExpansionHubsLIS getExpansionHubs() {
        return dcExpansionHubsLIS;
    }

    public void setDCExpansionHubsLIS(DCExpansionHubsLIS dExpansionHubsLIS) {
        this.dcExpansionHubsLIS = dExpansionHubsLIS;
    }

    public RobotStateEstimator getRobotStateEstimator() {
        return robotStateEstimator;
    }

    public void setRobotStateEstimator(RobotStateEstimator robotStateEstimator) {
        this.robotStateEstimator = robotStateEstimator;
    }

    public Drive getDrive() {
        return drive;
    }

    public void setDrive(Drive drive) {
        this.drive = drive;
    }

    public DCLiftSubsystem getDCLiftSubsystem() {
        return DCliftSubsystem;
    }

    public void setDCLiftSubsystem(DCLiftSubsystem DCliftSubsystem){
        this.DCliftSubsystem = DCliftSubsystem;
    }

    public DCAgitatorSubsystem getDCAgitatorSubsystem() {
        return DCagitatorSubsystem;
    }

    public void setDCAgitatorSubsystem(DCAgitatorSubsystem DCajadatorSubsystem){
        this.DCagitatorSubsystem = DCajadatorSubsystem;
    }
    public DCServoSubsystem getDCServoSubsystem() {
        return DCservoSubsystem;
    }

    public void setDCServoSubsystem(DCServoSubsystem DCservoSubsystem){
        this.DCservoSubsystem = DCservoSubsystem;
    }

    public DCShooterSubsystem getDCShooterSubsystem() {
        return DCshooterSubsystem;
    }

    public void setDCShooterSubsystem(DCShooterSubsystem DCshooterSubsystem){
        this.DCshooterSubsystem = DCshooterSubsystem;
    }
    public DCIntakeSubsystem getDCintakeSubsystem() {
        return DCintakeSubsystem;
    }

    public void setDCintakeSubsystem(DCIntakeSubsystem DCintakeSubsystem){
        this.DCintakeSubsystem = DCintakeSubsystem;
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

    public Pose2d getRobotPose() {
        return getRobotStateEstimator().getPose();
    }

    public double getRobotSpeed() {
        return getRobotStateEstimator().getVelocityPose().getTranslation().norm() +
                Math.abs(getRobotStateEstimator().getVelocityPose().getRotation().getRadians());
    }
}