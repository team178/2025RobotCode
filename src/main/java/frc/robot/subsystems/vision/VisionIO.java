package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean connected = false;
        public Pose2d poseEstimate = new Pose2d();
        public Pose2d poseEstimate2 = new Pose2d();
        public double[] stdDevs = new double[2];
        public double[] stdDevs2 = new double[2];
        public double lastTimestamp = 0;
    }

    public default void updateInputs(VisionIOInputs inputs) {}
}
