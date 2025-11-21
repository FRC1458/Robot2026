package frc.robot.auto;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Method;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * A class to select autos
 */
public class AutoSelector {

    @Retention(RetentionPolicy.RUNTIME) 
    @Target(ElementType.METHOD)
    /** An autonomous routine command. */
    public static @interface Auto {
        String name() default "";
    }
    
    private final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();

    public AutoSelector() {
        Method[] autos = AutoRoutines.class.getMethods();
        // Warning: dark and evil magic below
        for (Method auto : autos) {
            if (auto.isAnnotationPresent(Auto.class)) {
                String name = auto.getAnnotation(Auto.class).name();
                if (name.equals("")) {
                    name = auto.getName();
                }

                if (auto.getReturnType().equals(Command.class)) {
                    chooser.addOption(name, () -> {
                        try {
                            return (Command) auto.invoke(null);
                        } catch (Exception e) {
                            return null;
                        }
                    });
                } else {
                    DriverStation.reportWarning(
                        "@Auto annotation for this element is not supported", true);
                }
            }
        }

        chooser.setDefaultOption("None", () -> null);
        SmartDashboard.putData(chooser);
    }

    /** Gets the auto selected from the SmartDashboard */
    public Command getAuto() {
        return chooser.getSelected().get();
    }
}
