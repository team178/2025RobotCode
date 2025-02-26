package frc.robot.subsystems.swerve;

public enum FieldZones {
    BLUE_CLOSE(0), BLUE_CLOSE_LEFT(300), BLUE_CLOSE_RIGHT(60),
    BLUE_FAR(180), BLUE_FAR_LEFT(240),   BLUE_FAR_RIGHT(120),

    RED_CLOSE,  RED_CLOSE_LEFT,  RED_CLOSE_RIGHT,
    RED_FAR,    RED_FAR_LEFT,    RED_FAR_RIGHT,

    OPPOSITE,
    ;

    private double angle;

    private FieldZones() {
        this(0);
    }
    
    private FieldZones(double degrees) {
        angle = degrees;
    }
}
