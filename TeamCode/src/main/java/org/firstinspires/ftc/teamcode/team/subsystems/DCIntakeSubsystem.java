package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;

/**
 * Intake subsystem for DC robot.
 * Manages the intake motor and state machine.
 */
public class DCIntakeSubsystem extends BaseSubsystem<DCIntakeStateMachine, DCIntakeStateMachine.State> {
    private final RevMotor intakeMotor;

    /**
     * Creates a new intake subsystem.
     *
     * @param intakeMotor The motor for the intake
     */
    public DCIntakeSubsystem(RevMotor intakeMotor) {
        super(new DCIntakeStateMachine(), "Intake Subsystem");
        this.intakeMotor = intakeMotor;
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0.0);
    }

    @Override
    public void update(double dt) {
        super.update(dt);
        intakeMotor.setPower(getState().getPower());
    }

    /**
     * Gets the intake motor.
     *
     * @return The intake motor
     */
    public RevMotor getIntakeMotor() {
        return intakeMotor;
    }
}