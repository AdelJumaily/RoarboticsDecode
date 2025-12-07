package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.annotations.PIDSVA;
import org.firstinspires.ftc.teamcode.lib.control.ControlConstants;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.team.DbgLog;
import org.firstinspires.ftc.teamcode.team.states.DCLiftStateMachine;

@PIDSVA(name = "Extend",
        P = 0.4,
        I = 0.0,
        D = 0.00,
        S = 0.05,
        V = (1 - 0.05) / 15.0,
        A = 0.0
)
@PIDSVA(name = "Retract",
        P = 0.08,
        I = 0.0,
        D = 0.0,
        S = 0.05,
        V = 1.0 / 55.0,
        A = 0.0
)
/**
 * Lift subsystem for DC robot.
 * Manages the lift motor with PID control and motion profiles.
 */
public class DCLiftSubsystem extends BaseSubsystem<DCLiftStateMachine, DCLiftStateMachine.State> {
    // Control constants are static as they're configuration (loaded from annotations)
    private static final ControlConstants EXTEND_CONTROL_CONSTANTS;
    private static final ControlConstants RETRACT_CONTROL_CONSTANTS;

    private final RevMotor liftMotor;
    private IMotionProfile extensionProfile;
    private double setpoint;
    private double desiredSetpoint;
    private double lastError;
    private double runningSum;

    static {
        //new Thread(ResidualVibrationReductionMotionProfilerGenerator::init).start();
        PIDSVA[] controllers = DCLiftSubsystem.class.getAnnotationsByType(PIDSVA.class);
        if(controllers.length == 2) {
            PIDSVA extendController;
            PIDSVA retractController;
            if(controllers[0].name().equals(DCLiftStateMachine.State.EXTEND.getName())) {
                extendController  = controllers[0];
                retractController = controllers[1];
            } else {
                extendController  = controllers[1];
                retractController = controllers[0];
            }

            EXTEND_CONTROL_CONSTANTS = new ControlConstants(
                    extendController.P(), extendController.I(), extendController.D(),
                    extendController.S(), extendController.V(), extendController.A()
            );

            RETRACT_CONTROL_CONSTANTS = new ControlConstants(
                    retractController.P(), retractController.I(), retractController.D(),
                    retractController.S(), retractController.V(), retractController.A()
            );
        } else {
            EXTEND_CONTROL_CONSTANTS  = new ControlConstants();
            RETRACT_CONTROL_CONSTANTS = new ControlConstants();
        }
    }

    /**
     * Creates a new lift subsystem.
     *
     * @param liftMotor The motor for the lift
     */
    public DCLiftSubsystem(RevMotor liftMotor) {
        super(new DCLiftStateMachine(null), "Lift Subsystem");
        this.liftMotor = liftMotor;
        this.setpoint = 0.0;
        this.desiredSetpoint = 0.0;
        this.lastError = 0.0;
        this.runningSum = 0.0;
        this.extensionProfile = null;
        
        // Set the state machine's reference to this subsystem
        getStateMachine().setDCLiftSubsystem(this);
    }

    /**
     * Resets the running sum for integral control.
     */
    public void resetRunningSum() {
        runningSum = 0.0;
    }

    @Override
    public void stop() {
        liftMotor.setPower(0.0);
        liftMotor.resetEncoder();
        setpoint = 0.0;
        desiredSetpoint = 0.0;
    }

    @Override
    public void update(double dt) {
        super.update(dt);

        double error = setpoint - liftMotor.getPosition();
        double setpointVelocity = 0.0;
        double setpointAcceleration = 0.0;
        
        if (extensionProfile != null && !extensionProfile.isDone()) {
            setpointVelocity = extensionProfile.getVelocity();
            setpointAcceleration = extensionProfile.getAcceleration();
        }

        runningSum += error * dt;
        
        double output;
        if (getState().equals(DCLiftStateMachine.State.EXTEND)) {
            output = EXTEND_CONTROL_CONSTANTS.getOutput(dt, error, lastError, runningSum, 
                    setpointVelocity, setpointAcceleration, false);
            output += EXTEND_CONTROL_CONSTANTS.kS();
        } else {
            output = RETRACT_CONTROL_CONSTANTS.getOutput(dt, error, lastError, runningSum, 
                    setpointVelocity, setpointAcceleration, true);
        }

        lastError = error;
        liftMotor.setPower(output);
    }

    /**
     * Extends the lift to the specified position.
     *
     * @param position The target position
     */
    public void extend(double position) {
        resetRunningSum();
        setSetpoint(position);
    }

    /**
     * Retracts the lift to home position.
     */
    public void retract() {
        resetRunningSum();
        setSetpoint(0.0);
        desiredSetpoint = 0.0;
    }

    /**
     * Checks if the lift is close to the setpoint.
     *
     * @param threshold The tolerance threshold
     * @return true if within threshold
     */
    public boolean closeToSetpoint(double threshold) {
        return Math.abs(setpoint - liftMotor.getPosition()) <= threshold;
    }

    /**
     * Sets the lift setpoint and updates state machine accordingly.
     *
     * @param setpoint The target setpoint
     */
    public void setSetpoint(double setpoint) {
        desiredSetpoint = setpoint;
        if (setpoint != this.setpoint && (extensionProfile == null || extensionProfile.isDone())) {
            if (setpoint != 0.0) {
                // Extending
                getStateMachine().updateState(DCLiftStateMachine.State.EXTEND);
                this.setpoint = setpoint;
            } else {
                // Retracting
                getStateMachine().updateState(DCLiftStateMachine.State.RETRACT);
                this.setpoint = setpoint;
            }
        }
    }

    // Getters
    public RevMotor getLiftMotor() {
        return liftMotor;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public double getDesiredSetpoint() {
        return desiredSetpoint;
    }

    public IMotionProfile getExtensionProfile() {
        return extensionProfile;
    }

    public void setExtensionProfile(IMotionProfile extensionProfile) {
        this.extensionProfile = extensionProfile;
    }

    public static ControlConstants getExtendControlConstants() {
        return EXTEND_CONTROL_CONSTANTS;
    }

    public static ControlConstants getRetractControlConstants() {
        return RETRACT_CONTROL_CONSTANTS;
    }
}
