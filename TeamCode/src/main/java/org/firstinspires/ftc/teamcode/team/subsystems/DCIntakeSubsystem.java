package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.DCIntakeStateMachine;

public class DCIntakeSubsystem implements ISubsystem<DCIntakeStateMachine, DCIntakeStateMachine.State> {
    private static DCIntakeStateMachine DCintakeStateMachine;
    private RevMotor intakeWheels;

    public DCIntakeSubsystem(RevMotor intakeMotor){
        setIntakeStateMachine(new DCIntakeStateMachine());
        setIntakeWheels(intakeMotor);
    }

    @Override
    public DCIntakeStateMachine getStateMachine() {
        return DCintakeStateMachine;
    }

    @Override
    public DCIntakeStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getIntakeWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getIntakeWheels().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Flywheel Subsystem";
    }

    private static void setIntakeStateMachine(DCIntakeStateMachine intakeSM) {
        DCIntakeSubsystem.DCintakeStateMachine = intakeSM;
    }

    private void setIntakeWheels(RevMotor intakeMotor){
        this.intakeWheels = intakeMotor;
    }
    private RevMotor getIntakeWheels(){
        return intakeWheels;
    }
}