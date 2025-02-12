package frc.robot.subsystems.manipulator;

public enum ManipulatorPosition {
    // POSITIONS USING RADIANS
    HOME("HOME", 0.0),
    INTAKE("INTAKE", 0.0),
    CARRY("CARRY", 0.0);

    public final String name;
    public final double angle;

    private ManipulatorPosition(String name, double angle) {
        this.name = name;
        this.angle = angle;
    }

    @Override
    public String toString() {
        return name;
    }
}