package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
        
        public double lastTimestampMT1 = 0;
        public double lastTimestampMT2 = 0;
        public PoseObservation lastPoseObservationMT1 = new PoseObservation(0, new Pose3d(), 0, 0, 0, PoseObservationType.MEGATAG_1, LimelightLocations.DEFAULT); // get atomic
        public PoseObservation lastPoseObservationMT2 = new PoseObservation(0, new Pose3d(), 0, 0, 0, PoseObservationType.MEGATAG_2, LimelightLocations.DEFAULT); // get atomic
        public PoseObservation[] poseObservations = new PoseObservation[0];
        
        public int[] tagIds = new int[0]; // stores all values in last robot code loop; if desired, add tagIds in just the last observation and/or tagIds per PoseObservation; does not store duplicates
        
        public double[] lastStdDevsMT1 = new double[2];
        public double[] lastStdDevsMT2 = new double[2];
        public double[][] stdDevsMT1 = new double[0][2];
        public double[][] stdDevsMT2 = new double[0][2];
    }

    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    public static record PoseObservation(
      double timestamp,
      Pose3d pose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      PoseObservationType type,
      LimelightLocations limelightLocation
    ) {}

    public static enum PoseObservationType {
        MEGATAG_1,
        MEGATAG_2
    }

    public default LimelightLocations getLimelightLocation() {
        return LimelightLocations.DEFAULT;
    }

    public default void updateOrientation() {}

    public default void updateInputs(VisionIOInputs inputs) {}
}
