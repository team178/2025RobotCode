package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.FieldConstants;

public enum FieldZones {
    BLUE_CLOSE(
        0,
        FieldConstants.blueCloseLeftReef,
        FieldConstants.blueCloseRightReef
    ),
    BLUE_CLOSE_RIGHT(
        60,
        FieldConstants.blueCloseLeftReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(60)),
        FieldConstants.blueCloseRightReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(60))
    ),
    BLUE_FAR_RIGHT(
        120,
        FieldConstants.blueCloseLeftReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(120)),
        FieldConstants.blueCloseRightReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(120))
    ),
    BLUE_FAR(
        180,
        FieldConstants.blueCloseLeftReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(180)),
        FieldConstants.blueCloseRightReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(180))
    ),
    BLUE_FAR_LEFT(
        240,
        FieldConstants.blueCloseLeftReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(240)),
        FieldConstants.blueCloseRightReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(240))
    ),
    BLUE_CLOSE_LEFT(
        300,
        FieldConstants.blueCloseLeftReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(300)),
        FieldConstants.blueCloseRightReef.rotateAround(FieldConstants.reefCenterBlue, Rotation2d.fromDegrees(300))
    ),

    RED_FAR(
        0,
        FieldConstants.redFarLeftReef,
        FieldConstants.redFarRightReef
    ),
    RED_FAR_LEFT(
        60,
        FieldConstants.redFarLeftReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(60)),
        FieldConstants.redFarRightReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(60))
    ),
    RED_CLOSE_LEFT(
        120,
        FieldConstants.redFarLeftReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(120)),
        FieldConstants.redFarRightReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(120))
    ),
    RED_CLOSE(
        180,
        FieldConstants.redFarLeftReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(180)),
        FieldConstants.redFarRightReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(180))
    ),
    RED_CLOSE_RIGHT(
        240,
        FieldConstants.redFarLeftReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(240)),
        FieldConstants.redFarRightReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(240))
    ),
    RED_FAR_RIGHT(
        300,
        FieldConstants.redFarLeftReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(300)),
        FieldConstants.redFarRightReef.rotateAround(FieldConstants.reefCenterRed, Rotation2d.fromDegrees(300))
    ),

    OPPOSITE,
    ;

    public final double angle;
    public final Pose2d leftReefPose;
    public final Pose2d rightReefPose;

    private FieldZones() {
        this(0);
    }
    
    private FieldZones(double degrees) {
        this(degrees, new Pose2d(), new Pose2d());
    }

    private FieldZones(double degrees, Pose2d leftReefPose, Pose2d rightReefPose) {
        angle = degrees;
        this.leftReefPose = leftReefPose;
        this.rightReefPose = rightReefPose;
    }

    public Rotation2d rotation() {
        return Rotation2d.fromDegrees(angle);
    }
}
