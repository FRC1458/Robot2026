package frc.robot.lib.util;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** TODO: i'm considering moving this to a folder where it makes sense... maybe a "simulation" folder? */
public class Mechanisms {
    private static Mechanisms ssmechInstance;
    public static Mechanisms getInstance() {
        if (ssmechInstance == null) {
            ssmechInstance = new Mechanisms();
        }
        return ssmechInstance;
    }

    public final Mechanism2d elevatorAndArm;
    public final MechanismRoot2d elevatorAndArmRoot;
    private Mechanisms() {
        elevatorAndArm = new Mechanism2d(1.0, 3.0);
        elevatorAndArmRoot = elevatorAndArm.getRoot("Root", 0.5, 0.1);
		SmartDashboard.putData("Mechanisms/Elevator and Arm", elevatorAndArm);
    }
}
