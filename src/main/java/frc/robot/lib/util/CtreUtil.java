package frc.robot.lib.util;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DriverStation;

public class CtreUtil {
    public static final int MAX_RETRIES = 10;

    public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
        StatusCode statusCode = StatusCode.OK;
        for (int i = 0; i < MAX_RETRIES; i++) {
            statusCode = function.get();
            if (statusCode == StatusCode.OK) {
                break;
            }
        }
        if (statusCode != StatusCode.OK) {
            DriverStation.reportError(
                "Error calling "
                    + function
                    + " on ctre device id "
                    + deviceId
                    + ": "
                    + statusCode,
                true);
        }
        return statusCode;
    }

    public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
    }

    public static StatusCode applyConfigurationNonBlocking(TalonFX motor, VoltageConfigs config) {
        return motor.getConfigurator().apply(config, 0.01);
    }
    
    public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
        return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
    }

    public static StatusCode applyFollower(int main, TalonFX motor, MotorAlignmentValue value) {
        return tryUntilOK(
            () -> motor.setControl(new Follower(main, value)), 
            motor.getDeviceID());
    }
}