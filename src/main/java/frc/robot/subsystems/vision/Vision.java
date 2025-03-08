package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private PoseEstimateConsumer consumer;
    private VisionIOWrapper[] wrappedIOs;

    private static final boolean useAtomic = false;

    public Vision(PoseEstimateConsumer consumer, VisionIO... ios) {
        this.consumer = consumer;
        wrappedIOs = new VisionIOWrapper[ios.length];
        for(int i = 0; i < ios.length; i++) {
            wrappedIOs[i] = new VisionIOWrapper(
                ios[i],
                new VisionIOInputsAutoLogged(),
                new Alert("Vision module " + ios[i].getLimelightLocation().name + " disconnected.", AlertType.kWarning)
            );
        }
    }

    @Override
    public void periodic() {
        for(VisionIOWrapper wrappedIO : wrappedIOs) {
            wrappedIO.io.updateOrientation();
        }
        // NetworkTableInstance.getDefault().flush(); // TODO check if need; increases network traffic, but is recommended?
        for(VisionIOWrapper wrappedIO : wrappedIOs) {
            wrappedIO.io.updateInputs(wrappedIO.inputs);
            Logger.processInputs("Vision/" + wrappedIO.io.getLimelightLocation().name, wrappedIO.inputs);

            wrappedIO.disconnectedAlert.set(!wrappedIO.inputs.connected);

            if(useAtomic) {
                consumer.accept(
                    wrappedIO.inputs.lastPoseObservationMT1.pose().toPose2d(),
                    wrappedIO.inputs.lastTimestampMT1,
                    convertStdDevs(wrappedIO.inputs.lastStdDevsMT1)
                );
                consumer.accept(
                    wrappedIO.inputs.lastPoseObservationMT2.pose().toPose2d(),
                    wrappedIO.inputs.lastTimestampMT2,
                    convertStdDevs(wrappedIO.inputs.lastStdDevsMT2)
                );
            } else {
                if(wrappedIO.inputs.stdDevsMT1.length + wrappedIO.inputs.stdDevsMT2.length != wrappedIO.inputs.poseObservations.length) {
                    System.out.println("Expected # of pose observations does not match lengths of standard deviations");
                }
                for(int i = 0; i < wrappedIO.inputs.stdDevsMT1.length; i++) {
                    if(wrappedIO.io.getLimelightLocation().equals(LimelightLocations.HIGH3)) {
                        consumer.accept(
                            wrappedIO.inputs.poseObservations[i].pose().toPose2d(),
                            wrappedIO.inputs.poseObservations[i].timestamp() * 1e-6,
                            convertStdDevs(wrappedIO.inputs.stdDevsMT1[i])
                        );
                    }
                }
                for(int i = 0; i < wrappedIO.inputs.stdDevsMT2.length; i++) {
                    if(!wrappedIO.io.getLimelightLocation().equals(LimelightLocations.HIGH3)) {
                        consumer.accept(
                            wrappedIO.inputs.poseObservations[wrappedIO.inputs.stdDevsMT1.length + i].pose().toPose2d(),
                            wrappedIO.inputs.poseObservations[wrappedIO.inputs.stdDevsMT1.length + i].timestamp() * 1e-6,
                            convertStdDevs(wrappedIO.inputs.stdDevsMT2[i])
                        );
                    }
                }
            }
        }
    }

    private static Matrix<N3, N1> convertStdDevs(double[] stdDevs) {
        return stdDevs.length == 2 ? VecBuilder.fill(stdDevs[0], stdDevs[0], stdDevs[1]) : VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    }

    @FunctionalInterface
    public static interface PoseEstimateConsumer {
        public void accept(Pose2d visionMeasurement, double timestamp, Matrix<N3,N1> stdDevs);
    }

    private static record VisionIOWrapper(VisionIO io, VisionIOInputsAutoLogged inputs, Alert disconnectedAlert) {}
}
