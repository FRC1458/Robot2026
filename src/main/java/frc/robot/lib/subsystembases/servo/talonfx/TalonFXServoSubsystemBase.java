package frc.robot.lib.subsystembases.servo.talonfx;

import frc.robot.lib.io.motor.talonfx.TalonFXInputs;
import frc.robot.lib.io.motor.talonfx.TalonFXInputsAutoLogged;
import frc.robot.lib.io.motor.talonfx.TalonFXMotorIO;
import frc.robot.lib.subsystembases.servo.ServoSubsystemBase;

public class TalonFXServoSubsystemBase 
    extends ServoSubsystemBase<
        TalonFXInputs,
        TalonFXInputsAutoLogged,
        TalonFXMotorIO> 
{
    public TalonFXServoSubsystemBase(TalonFXServoSubsystemConstants constants) {
        super(constants);

        inputs = new TalonFXInputsAutoLogged();
        io = new TalonFXMotorIO(constants.main, constants.config, getName());
    }
}
