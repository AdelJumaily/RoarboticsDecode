package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.team.states.DCOuttakeStateMachine;

public class DCOuttakeSubsystem implements ISubsystem<DCOuttakeStateMachine, DCOuttakeStateMachine.State> {
    private static DCOuttakeStateMachine DCouttakeStateMachine;
    private RevMotor outtakeWheels;

    public DCOuttakeSubsystem(RevMotor outtakeMotor){
        setOuttakeStateMachine(new DCOuttakeStateMachine());
        setOuttakeWheels(outtakeMotor);
    }

    @Override
    public DCOuttakeStateMachine getStateMachine() {
        return DCouttakeStateMachine;
    }

    @Override
    public DCOuttakeStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {
        getOuttakeWheels().setPower(0d);
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getOuttakeWheels().setPower(getState().getPower());
    }

    @Override
    public String getName() {
        return "Outtle Flywheel Subsystem";
    }

    private static void setOuttakeStateMachine(DCOuttakeStateMachine outtakeSM) {
        DCOuttakeSubsystem.DCouttakeStateMachine = outtakeSM;
    }

    private void setOuttakeWheels(RevMotor outtakeMotor){
        this.outtakeWheels = outtakeMotor;
    }
    private RevMotor getOuttakeWheels(){
        return outtakeWheels;
    }
}