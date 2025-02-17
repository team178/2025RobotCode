package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private VisionConsumer consumer;
    private VisionIO[] ios;
    private VisionIOInputsAutoLogged[] inputs;
    private Alert[] disconnectedAlerts;

    public Vision(VisionConsumer consumer, VisionIO... ios) {
        this.consumer = consumer;
        this.ios = ios;
        
        inputs = new VisionIOInputsAutoLogged[ios.length];

        disconnectedAlerts = new Alert[ios.length];
        for (int i = 0; i < ios.length; i++) {
            disconnectedAlerts[i] = new Alert("Vision module " + Integer.toString(i) + " disconnected.", AlertType.kWarning);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < ios.length; i++) {
            ios[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);

            disconnectedAlerts[i].set(!inputs[i].connected);

            consumer.sendMeasurement(
                inputs[i].poseEstimate,
                Timer.getFPGATimestamp(), // TODO replace with timestamp from inputs
                VecBuilder.fill(inputs[i].stdDevs[0], inputs[i].stdDevs[0], inputs[i].stdDevs[1])
            );

            consumer.sendMeasurement(
                inputs[i].poseEstimate2,
                Timer.getFPGATimestamp(),
                VecBuilder.fill(inputs[i].stdDevs2[0], inputs[i].stdDevs2[0], inputs[i].stdDevs2[1])
            );
        }
    }

    @FunctionalInterface
    public static interface VisionConsumer {
        public void sendMeasurement(Pose2d visionMeasurement, double timestamp, Matrix<N3,N1> stdDevs);
    }
}
