package org.firstinspires.ftc.teamcode.team;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.team.auto.DCBaseLIS;

import java.util.Arrays;

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
public abstract class DCTeleopRobotLIS extends Robot {
    private TimeProfiler matchRuntime;
    protected DCBaseLIS drive;

    @Override
    public void init() {
        super.init();
        setMatchRuntime(new TimeProfiler(false));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        drive.getExpansionHubs().start();
        drive.robot.getDrive().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
        getMatchRuntime().start();
    }

    @Override
    public void loop() {
        super.loop();
        drive.getExpansionHubs().update(getDt());
        drive.update();
        drive.robot.getDCIntakeSubsystem().update(getDt());
        drive.robot.getDCShooterSubsystem().update(getDt());
        drive.robot.getDCLiftSubsystem().update(getDt());
    }

    @Override
    public void stop() {
        super.stop();
        drive.getExpansionHubs().stop();
        drive.robot.getDCLiftSubsystem().stop();
        drive.robot.getDCShooterSubsystem().stop();
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

}