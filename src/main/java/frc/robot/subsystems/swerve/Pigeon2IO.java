package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.SwerveConstants;

public class Pigeon2IO implements GyroIO {
    private Pigeon2 pigeon;

    public Pigeon2IO() {
        pigeon = new Pigeon2(SwerveConstants.kPigeonCANID);
        pigeon.reset();
        pigeon.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.yawPosition = Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }
}
