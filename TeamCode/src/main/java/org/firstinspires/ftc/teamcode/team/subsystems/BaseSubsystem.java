package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.team.states.IState;

/**
 * Base class for all robot subsystems.
 * Provides common functionality and reduces code duplication.
 * 
 * This class eliminates the need for static state and provides
 * a consistent pattern for all subsystems.
 *
 * @param <TStateMachine> The state machine type for this subsystem
 * @param <TState> The state enum type
 */
public abstract class BaseSubsystem<TStateMachine extends IState<TState>, TState extends Enum<TState> & Namable> 
        implements ISubsystem<TStateMachine, TState> {
    
    protected final TStateMachine stateMachine;
    protected final String subsystemName;

    /**
     * Constructor for base subsystem.
     *
     * @param stateMachine The state machine instance for this subsystem
     * @param subsystemName The display name of this subsystem
     */
    protected BaseSubsystem(TStateMachine stateMachine, String subsystemName) {
        this.stateMachine = stateMachine;
        this.subsystemName = subsystemName;
    }

    @Override
    public TStateMachine getStateMachine() {
        return stateMachine;
    }

    @Override
    public TState getState() {
        return stateMachine.getState();
    }

    @Override
    public String getName() {
        return subsystemName;
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {
        telemetry.addData(getName() + " State", getState());
    }

    /**
     * Subsystems should override this to implement their specific update logic.
     * The base implementation just updates the state machine.
     */
    @Override
    public void update(double dt) {
        stateMachine.update(dt);
    }

    /**
     * Subsystems should override this to implement their specific start logic.
     */
    @Override
    public void start() {
        // Default: do nothing
    }

    /**
     * Subsystems should override this to implement their specific stop logic.
     */
    @Override
    public void stop() {
        // Default: do nothing
    }
}

