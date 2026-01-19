package org.firstinspires.ftc.teamcode.team.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.drivers.RevServo;
import org.firstinspires.ftc.teamcode.team.states.DCServoStateMachine;

public class DCServoSubsystem implements ISubsystem<DCServoStateMachine, DCServoStateMachine.State> {
    private static DCServoStateMachine DCservoStateMachine;
    private RevServo liftServo;


    public DCServoSubsystem(RevServo liftServo){
        setDCServoStateMachine(new DCServoStateMachine());
        setGripperServo(liftServo);
    }

    @Override
    public DCServoStateMachine getStateMachine() {
        return DCservoStateMachine;
    }

    @Override
    public DCServoStateMachine.State getState() {
        return getStateMachine().getState();
    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }

    @Override
    public String getName() {
        return "Servo Subsystem";
    }

    @Override
    public void writeToTelemetry(Telemetry telemetry) {

    }

    @Override
    public void update(double dt) {
        getStateMachine().update(dt);
        getliftServo().setPosition(getState().getPosition());
    }

    public static void setDCServoStateMachine(DCServoStateMachine DCServoStateMachine) {
        DCServoSubsystem.DCservoStateMachine = DCServoStateMachine;
    }

    public RevServo getliftServo() {
        return liftServo;
    }


    public void setGripperServo(RevServo liftServo) {
        this.liftServo = liftServo;
    }
}