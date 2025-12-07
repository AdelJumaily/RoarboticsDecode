package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;

/**
 * Shooter subsystem for DC robot.
 * Manages the shooter motor and state machine.
 */
public class DCShooterSubsystem extends BaseSubsystem<DCShooterStateMachine, DCShooterStateMachine.State> {
    private final RevMotor shooterMotor;

    /**
     * Creates a new shooter subsystem.
     *
     * @param shooterMotor The motor for the shooter
     */
    public DCShooterSubsystem(RevMotor shooterMotor) {
        super(new DCShooterStateMachine(), "Shooter Subsystem");
        this.shooterMotor = shooterMotor;
    }

    @Override
    public void stop() {
        shooterMotor.setPower(0.0);
    }

    @Override
    public void update(double dt) {
        super.update(dt);
        shooterMotor.setPower(getState().getPower());
    }

    /**
     * Gets the shooter motor.
     *
     * @return The shooter motor
     */
    public RevMotor getShooterMotor() {
        return shooterMotor;
    }
}