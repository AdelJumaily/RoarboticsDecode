package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.DCAgitatorStateMachine;

public class DCAgitatorSubsystem implements ISubsystem<DCAgitatorStateMachine, DCAgitatorStateMachine.State> {
    private static DCAgitatorStateMachine DCagitatorStateMachine;
    private RevMotor AgitatorWheels;

    public DCAgitatorSubsystem(RevMotor AjadatorMotor){
        setAjadatorStateMachine(new DCAgitatorStateMachine());
        setAgitatorWheels(AjadatorMotor);
    }

    @Override
    public DCAgitatorStateMachine getStateMachine() {
        return DCagitatorStateMachine;
    }

    @Override
    public DCAgitatorStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getAgitatorWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getAgitatorWheels().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Flywheel Subsystem";
    }

    private static void setAjadatorStateMachine(DCAgitatorStateMachine AjadatorSM) {
        DCAgitatorSubsystem.DCagitatorStateMachine = AjadatorSM;
    }

    private void setAgitatorWheels(RevMotor AjadatorMotor){
        this.AgitatorWheels = AjadatorMotor;
    }
    private RevMotor getAgitatorWheels(){
        return AgitatorWheels;
    }
}