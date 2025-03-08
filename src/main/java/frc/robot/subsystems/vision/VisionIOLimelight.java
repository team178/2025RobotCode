package frc.robot.subsystems.vision;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

public class VisionIOLimelight implements VisionIO {
    private DoubleArrayPublisher orientationPublisher;
    private DoubleSubscriber txSubscriber;            // horizontal offset of target from crosshair     |  -27 to 27 degrees
    private DoubleSubscriber tySubscriber;            // vertical offset of target from crosshair       | -20.5 to 20.5 degrees
    // [x, y, z, roll, pitch, yaw, total latency, tag count, span, avg tag distance, avg tag area, (per raw tag data), id, txnc, tync, ta, distToCamera, distToRobot, ambiguity, ...]
    // [pose information(11), raw fiducial data(7n)]
    // [m, m, m, deg, deg, deg, ms, count, ", m?, % of image, ...]
    private DoubleArraySubscriber dataMT1subscriber;
    private DoubleArraySubscriber dataMT2Subscriber;

    private LimelightLocations limelightLocation;
    private Supplier<Rotation2d> yawSupplier;

    public VisionIOLimelight(Supplier<Rotation2d> yawSupplier) {
        this(LimelightLocations.DEFAULT, yawSupplier);
    }

    /**
     * 
     * @param hostname The name of the limelight. (used for logging + value retrieval) 
     * @param yawSupplier The current estimated robot yaw rotation; use gyro heading information
     */
    public VisionIOLimelight(LimelightLocations location, Supplier<Rotation2d> yawSupplier) {
        NetworkTable ioNT = NetworkTableInstance.getDefault().getTable(location.name); // hostname defaults to "limelight" in limelight config
        limelightLocation = location;

        orientationPublisher = ioNT.getDoubleArrayTopic("robot_orientation_set").publish();
        txSubscriber = ioNT.getDoubleTopic("tx").subscribe(0);
        tySubscriber = ioNT.getDoubleTopic("ty").subscribe(0);
        dataMT1subscriber = ioNT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[8]);
        dataMT2Subscriber = ioNT.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[8]);

        this.yawSupplier = yawSupplier;
    }

    @Override
    public LimelightLocations getLimelightLocation() {
        return limelightLocation;
    }

    @Override
    public void updateOrientation() {
        // if desired can add yaw rate
        orientationPublisher.accept(new double[] {yawSupplier.get().getDegrees(), 0, 0, 0, 0, 0}); // [yaw, yawrate, pitch, pitchrate, roll, rollrate] deg or deg/s
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // if the milliseconds since the last update is under 250, the device is connected (both methods return μs) (RobotController returns μs, Timer returns s)
        inputs.connected = ((RobotController.getFPGATime() - txSubscriber.getLastChange()) * 1e-3) < 250;

        inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));

        // update last pose observations
        TimestampedDoubleArray lastRawMT1Data = dataMT1subscriber.getAtomic();
        TimestampedDoubleArray lastRawMT2Data = dataMT2Subscriber.getAtomic();
        inputs.lastTimestampMT1 = lastRawMT1Data.timestamp;
        inputs.lastTimestampMT2 = lastRawMT2Data.timestamp;
        if(lastRawMT1Data.value.length >= 11) inputs.lastPoseObservationMT1 = parsePosition(lastRawMT1Data.value, lastRawMT1Data.timestamp * 1e-6, false);
        if(lastRawMT2Data.value.length >= 11) inputs.lastPoseObservationMT2 = parsePosition(lastRawMT2Data.value, lastRawMT2Data.timestamp * 1e-6, true);
        if(lastRawMT1Data.value.length >= 11) {
            int tagCountMT1 = (int) lastRawMT1Data.value[7];
            double averageTagDistanceMT1 = lastRawMT1Data.value[9];
            inputs.lastStdDevsMT1 = calculateStdDevs(averageTagDistanceMT1, tagCountMT1, false);
        }
        if(lastRawMT2Data.value.length >= 11) {
            int tagCountMT2 = (int) lastRawMT2Data.value[7];
            double averageTagDistanceMT2 = lastRawMT2Data.value[9];
            inputs.lastStdDevsMT2 = calculateStdDevs(averageTagDistanceMT2, tagCountMT2, true);
        }

        // update all pose observations in last robot code loop
        TimestampedDoubleArray[] subscriberDataMT1 = Arrays.stream(dataMT1subscriber.readQueue()).filter(element -> element.value.length > 11).toArray(TimestampedDoubleArray[]::new);
        TimestampedDoubleArray[] subscriberDataMT2 = Arrays.stream(dataMT2Subscriber.readQueue()).filter(element -> element.value.length > 11).toArray(TimestampedDoubleArray[]::new);
        PoseObservation[] poseObservations = new PoseObservation[subscriberDataMT1.length + subscriberDataMT2.length];
        Set<Integer> tagIds = new HashSet<>();
        double[][] stdDevsMT1 = new double[subscriberDataMT1.length][2];
        double[][] stdDevsMT2 = new double[subscriberDataMT2.length][2];
        for(int i = 0; i < subscriberDataMT1.length; i++) {
            TimestampedDoubleArray element = subscriberDataMT1[i];
            poseObservations[i] = parsePosition(element.value, element.timestamp, false);
            int tagCount = (int) element.value[7];
            int expectedLength = 11 + 7 * tagCount;
            if(element.value.length == expectedLength) {
                for(int j = 0; j < tagCount; j++) {
                    tagIds.add((int) element.value[11 + 0 + j * 7]);
                }
            }
            double averageTagDistance = element.value[9];
            stdDevsMT1[i] = calculateStdDevs(averageTagDistance, tagCount, false);
        }
        for(int i = 0; i < subscriberDataMT2.length; i++) {
            TimestampedDoubleArray element = subscriberDataMT2[i];
            poseObservations[subscriberDataMT1.length + i] = parsePosition(element.value, element.timestamp, true);
            int tagCount = (int) element.value[7];
            int expectedLength = 11 + 7 * tagCount;
            if(element.value.length == expectedLength) {
                for(int j = 0; j < tagCount; j++) {
                    tagIds.add((int) element.value[11 + 0 + j * 7]);
                }
            }
            double averageTagDistance = element.value[9];
            stdDevsMT2[i] = calculateStdDevs(averageTagDistance, tagCount, true);
        }
        inputs.poseObservations = poseObservations;
        inputs.tagIds = tagIds.stream().mapToInt(Number::intValue).toArray();
        inputs.stdDevsMT1 = stdDevsMT1;
        inputs.stdDevsMT2 = stdDevsMT2;
    }

    private PoseObservation parsePosition(double[] rawMegatagData, double timestampSeconds, boolean isMegaTag2) {
        // 11(pose info) + 7n(raw fiducials data, at least 1 tag)
        if(rawMegatagData.length < 11 + 7) {
            System.out.println("Invalid megatag data for parsing");
            return parsePosition(new double[11 + 7], Timer.getFPGATimestamp(), false);
        }
        double x = rawMegatagData[0];
        double y = rawMegatagData[1];
        double z = rawMegatagData[2];
        double roll = rawMegatagData[3];
        double pitch = rawMegatagData[4];
        double yaw = rawMegatagData[5];
        double latency = rawMegatagData[6];
        int tagCount = (int) rawMegatagData[7];
        double averageTagDistance = rawMegatagData[9];

        roll = Units.degreesToRadians(roll);
        pitch = Units.degreesToRadians(pitch);
        yaw = Units.degreesToRadians(yaw);

        double observationTimestamp = timestampSeconds - latency * 1e-3;
        Pose3d pose = new Pose3d(x, y, z, new Rotation3d(roll, pitch, yaw));
        double ambiguity = isMegaTag2 || rawMegatagData.length != 11 + 7 ? 0 : rawMegatagData[17];

        return new PoseObservation(observationTimestamp, pose, ambiguity, tagCount, averageTagDistance, isMegaTag2 ? PoseObservationType.MEGATAG_2 : PoseObservationType.MEGATAG_1, limelightLocation);
    }

    private double[] calculateStdDevs(double avgTagDist, int tagCount, boolean isMegaTag2) {
        double stdDevFactor = Math.pow(avgTagDist, 2) / tagCount;
        double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

        linearStdDev *= limelightLocation.linearStdDevFactor;
        angularStdDev *= limelightLocation.angularStdDevFactor;
        if(isMegaTag2) {
            linearStdDev *= VisionConstants.linearStdDevMT2Factor;
            angularStdDev *= VisionConstants.angularStdDevMT2Factor;
        }

        return new double[] {linearStdDev, angularStdDev};
    }
}
