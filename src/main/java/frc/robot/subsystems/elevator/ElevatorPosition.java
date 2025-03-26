package frc.robot.subsystems.elevator;

public enum ElevatorPosition {
    HOME("Home", 0.005),
    L1("L1", 0.372),
    L2("L2", 0.452),
    L3("L3", 0.612),
    // L4("L4", 0.0),
    ;

    public final String name;
    public final double height;

    public final static double homeCoralPos = 0.073;

    private ElevatorPosition(String name, double height) {
        this.name = name;
        this.height = height;
    }

    @Override
    public String toString() {
        return name;
    }
}
