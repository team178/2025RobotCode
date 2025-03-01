package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;

public enum FieldZones {
    BLUE_CLOSE(0),   BLUE_CLOSE_LEFT(300), BLUE_CLOSE_RIGHT(60),
    BLUE_FAR(180),   BLUE_FAR_LEFT(240),   BLUE_FAR_RIGHT(120),

    RED_CLOSE(180),  RED_CLOSE_LEFT(120),  RED_CLOSE_RIGHT(240),
    RED_FAR(0),      RED_FAR_LEFT(60),     RED_FAR_RIGHT(300),

    OPPOSITE,
    ;

    public final double angle;

    private FieldZones() {
        this(0);
    }
    
    private FieldZones(double degrees) {
        angle = degrees;
    }

    public Rotation2d rotation() {
        return Rotation2d.fromDegrees(angle);
    }
}
