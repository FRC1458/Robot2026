package frc.robot.lib.io.motor.followertalonfx;

import org.littletonrobotics.junction.LogTable;
import frc.robot.lib.io.motor.talonfx.TalonFXInputsAutoLogged;

public class FollowerTalonFXInputs extends TalonFXInputsAutoLogged {
    TalonFXInputsAutoLogged[] inputs;
    String[] names;
    int size;

    public FollowerTalonFXInputs(int size) {
        this.size = size;
        names = new String[size];
        inputs = new TalonFXInputsAutoLogged[size];
        for (int i = 0; i < size; i++) {
            inputs[i] = new TalonFXInputsAutoLogged();
            names[i] = "Follower " + i;
        }
    }

    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        for (int i = 0; i < size; i++) {
            inputs[i].toLog(table.getSubtable(names[i]));
        }
    }

    @Override
    public void fromLog(LogTable table) {
        super.fromLog(table);
        for (int i = 0; i < size; i++) {
            inputs[i].fromLog(table.getSubtable(names[i]));
        }
    }
}
