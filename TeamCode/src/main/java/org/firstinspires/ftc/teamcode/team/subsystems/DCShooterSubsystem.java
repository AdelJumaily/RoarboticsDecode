package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.DCShooterStateMachine;

public class DCShooterSubsystem implements ISubsystem<DCShooterStateMachine, DCShooterStateMachine.State> {
    private static DCShooterStateMachine DCShooterStateMachine;
    private RevMotor ShooterWheels;

    public DCShooterSubsystem(RevMotor ShooterMotor){
        setShooterStateMachine(new DCShooterStateMachine());
        setShooterWheels(ShooterMotor);
    }

    @Override
    public DCShooterStateMachine getStateMachine() {
        return DCShooterStateMachine;
    }

    @Override
    public DCShooterStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getShooterWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getShooterWheels().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Outtle Flywheel Subsystem";
    }

    private static void setShooterStateMachine(DCShooterStateMachine ShooterSM) {
        DCShooterSubsystem.DCShooterStateMachine = ShooterSM;
    }

    private void setShooterWheels(RevMotor ShooterMotor){
        this.ShooterWheels = ShooterMotor;
    }
    private RevMotor getShooterWheels(){
        return ShooterWheels;
    }
}