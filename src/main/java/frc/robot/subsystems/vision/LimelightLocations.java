package frc.robot.subsystems.vision;

public enum LimelightLocations {
    FRONT3("limelight-front", 1, 1),
    HIGH3("limelight-high", 1.5, 1.5),
    SIDE2("limelight-side", 2, 2),
    DEFAULT("limelight", 1, 1)
    ;

    public final String name;
    public final double linearStdDevFactor;
    public final double angularStdDevFactor;

    private LimelightLocations(String name, double linearStdDevFactor, double angularStdDevFactor) {
        this.name = name;
        this.linearStdDevFactor = linearStdDevFactor;
        this.angularStdDevFactor = angularStdDevFactor;
    }

    @Override
    public String toString() {
        return name;
    }
}
