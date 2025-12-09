package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.DCAjadatorStateMachine;

public class DCAjadatorSubsystem implements ISubsystem<DCAjadatorStateMachine, DCAjadatorStateMachine.State> {
    private static DCAjadatorStateMachine DCajadatorStateMachine;
    private RevMotor AjadatorWheels;

    public DCAjadatorSubsystem(RevMotor AjadatorMotor){
        setAjadatorStateMachine(new DCAjadatorStateMachine());
        setAjadatorWheels(AjadatorMotor);
    }

    @Override
    public DCAjadatorStateMachine getStateMachine() {
        return DCajadatorStateMachine;
    }

    @Override
    public DCAjadatorStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getAjadatorWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getAjadatorWheels().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Flywheel Subsystem";
    }

    private static void setAjadatorStateMachine(DCAjadatorStateMachine AjadatorSM) {
        DCAjadatorSubsystem.DCajadatorStateMachine = AjadatorSM;
    }

    private void setAjadatorWheels(RevMotor AjadatorMotor){
        this.AjadatorWheels = AjadatorMotor;
    }
    private RevMotor getAjadatorWheels(){
        return AjadatorWheels;
    }
}