package frc.robot.lib.util;

public record CanDevice(int id, String bus) {
    public static CanDevice fromId(int id) {
        return new CanDevice(id, "CV");
    }
}
