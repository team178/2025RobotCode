package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.VisionConstants;

public class VisionIOLimelight implements VisionIO {
    /* IO metadata */
    private int index;
    private Supplier<Rotation2d> yawSupplier;

    private NetworkTable ioNT;
    private DoubleArrayPublisher orientationPublisher;
    /* Listeners for limelight data from networktables */
    private DoubleSubscriber tlSubscriber;            // media stream latency 
    // private DoubleSubscriber txSubscriber;            // horizontal offset     |  -27  to  27  degrees
    // private DoubleSubscriber tySubscriber;            // vertical offset       | -20.5 to 20.5 degrees
    // private DoubleSubscriber taSubscriber;            // target area           | % of fov taken
    /* The real bulk operation 💪 */ 
    private DoubleArraySubscriber megatagSubscriber;  // robot (posi/rota)tion | (x, y, z) meters, (roll, pitch yaw) deg
    private DoubleArraySubscriber megatag2Subscriber; // 

    /**
     * 
     * @param hostname The name of the limelight. (used for logging + value retrieval) 
     * @param yawSupplier The current estimated robot yaw rotation.
     */
    public VisionIOLimelight(String hostname, int index, Supplier<Rotation2d> yawSupplier) {
        this.index = index;
        this.yawSupplier = yawSupplier;

        ioNT = NetworkTableInstance.getDefault().getTable(hostname); // hostname defaults to "limelight" in limelight config
        orientationPublisher = ioNT.getDoubleArrayTopic("robot_orientation_set").publish();
        tlSubscriber = ioNT.getDoubleTopic("tl").subscribe(0);
        // txSubscriber = ioNT.getDoubleTopic("tx").subscribe(0);
        // tySubscriber = ioNT.getDoubleTopic("ty").subscribe(0);
        // taSubscriber = ioNT.getDoubleTopic("ta").subscribe(0);
        megatagSubscriber = ioNT.getDoubleArrayTopic("botpose_wpiblue").subscribe(new double[6]);
        megatag2Subscriber = ioNT.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[6]);
    }

    private Pose2d parsePosition(double[] megatagRobotPose, boolean isMegaTag2) {
        // weird coordinate system according to docs
        double x = megatagRobotPose[0];
        double y = -megatagRobotPose[1];
        // double z = megatagRobotPose[2];
        // double roll = megatagRobotPose[3];
        // double pitch = megatagRobotPose[4];
        double yaw = megatagRobotPose[5];

        Translation2d translation = new Translation2d(x, y);
        Rotation2d rotation = new Rotation2d(yaw);

        return new Pose2d(translation, rotation);
    }

    /** i dont know what im doing */
    private double[] calculateStdDevs(double avgTagDist, int tagCount, boolean isMegaTag2) {
        double stdDevFactor = Math.pow(avgTagDist, 2) / tagCount;
        double linearStdDev = VisionConstants.linearStdDevBaseline * stdDevFactor;
        double angularStdDev = VisionConstants.angularStdDevBaseline * stdDevFactor;

        if (index < VisionConstants.cameraStdDevFactors.length) {
            linearStdDev *= VisionConstants.cameraStdDevFactors[index];
            angularStdDev *= VisionConstants.cameraStdDevFactors[index];
        }

        if (isMegaTag2) {
            linearStdDev *= VisionConstants.linearStdDevMegatag2Factor;
            angularStdDev *= VisionConstants.angularStdDevMegatag2Factor;
        }

        return new double[] { linearStdDev, angularStdDev };
    } 

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        // if the milliseconds since the last update is under 250, the device is connected
        inputs.connected = ((RobotController.getFPGATime() - tlSubscriber.getLastChange()) / 1000) < 250;
 
        // update robot orientations
        orientationPublisher.accept(new double[] { yawSupplier.get().getDegrees(), 0, 0, 0, 0, 0 });

        // update pose estimates
        double[] mtResult = megatagSubscriber.get();
        double[] mt2Result = megatag2Subscriber.get();

        inputs.poseEstimate = parsePosition(mtResult, false);
        inputs.poseEstimate2 = parsePosition(mt2Result, true);

        inputs.stdDevs = calculateStdDevs(/* avg dist */ mtResult[9], /* count */ (int) mtResult[7], false);
        inputs.stdDevs2 = calculateStdDevs(/* avg dist */ mt2Result[9], /* count */ (int) mt2Result[7], true);

        inputs.lastTimestamp = megatagSubscriber.getLastChange();
    }
}
