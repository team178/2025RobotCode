package frc.robot.subsystems.elevator;

public enum ElevatorPosition {
    HOME("Home", 0.005),
    L1("L1", 0.327),
    L2("L2", 0.452),
    L3("L3", 0.609),
    // L4("L4", 0.0),
    ;

    public final String name;
    public final double height;

    private ElevatorPosition(String name, double height) {
        this.name = name;
        this.height = height;
    }

    @Override
    public String toString() {
        return name;
    }
}
