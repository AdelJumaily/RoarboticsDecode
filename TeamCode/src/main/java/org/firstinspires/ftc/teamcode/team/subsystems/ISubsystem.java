package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.TelemetryWritable;
import org.firstinspires.ftc.teamcode.lib.util.Updatable;
import org.firstinspires.ftc.teamcode.team.states.IState;

/**
 * Interface for robot subsystems.
 * All subsystems must implement this interface.
 *
 * @param <M> The state machine type
 * @param <S> The state enum type
 */
public interface ISubsystem<M extends IState<S>, S extends Enum<S> & Namable> 
        extends TelemetryWritable, Updatable, Namable {
    
    /**
     * Gets the state machine for this subsystem.
     *
     * @return The state machine instance
     */
    M getStateMachine();

    /**
     * Gets the current state of this subsystem.
     *
     * @return The current state
     */
    S getState();

    /**
     * Called when the subsystem should start.
     * Typically called during robot initialization.
     */
    void start();

    /**
     * Called when the subsystem should stop.
     * Typically called when the robot is disabled.
     */
    void stop();
}

