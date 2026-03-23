package frc.robot.lib.io;

public interface IO<I extends Inputs> {
    void readInputs(I inputs);
}
