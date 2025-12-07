package org.firstinspires.ftc.teamcode.team.states;

import org.firstinspires.ftc.teamcode.lib.util.Namable;
import org.firstinspires.ftc.teamcode.lib.util.TelemetryWritable;
import org.firstinspires.ftc.teamcode.lib.util.Updatable;

/**
 * Interface for state machines.
 * Defines the contract for managing robot subsystem states.
 *
 * @param <E> The state enum type
 */
public interface IState<E extends Enum<E> & Namable> extends TelemetryWritable, Updatable, Namable {
    /**
     * Updates the desired state of the state machine.
     *
     * @param state The new desired state
     */
    void updateState(E state);

    /**
     * Checks if the current state goal has been reached.
     *
     * @return true if the goal has been reached
     */
    boolean hasReachedStateGoal();

    /**
     * Checks if a specific state goal has been reached.
     *
     * @param state The state to check
     * @return true if the goal has been reached
     */
    boolean hasReachedStateGoal(E state);

    /**
     * Checks if the state machine is attempting to change states.
     *
     * @return true if a state change is in progress
     */
    boolean attemptingStateChange();

    /**
     * Gets the current state.
     *
     * @return The current state
     */
    E getState();

    /**
     * Gets the desired state.
     *
     * @return The desired state
     */
    E getDesiredState();
}

