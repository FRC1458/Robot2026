package frc.robot.lib.sim;

public class Mechanisms {
    private static Mechanisms ssmechInstance;
    public static Mechanisms getInstance() {
        if (ssmechInstance == null) {
            ssmechInstance = new Mechanisms();
        }
        return ssmechInstance;
    }
}
