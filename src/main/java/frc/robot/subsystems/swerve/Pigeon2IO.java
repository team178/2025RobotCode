package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Constants.SwerveConstants;

public class Pigeon2IO implements GyroIO {
    private Pigeon2 pigeon;
    private StatusSignal<Angle> yaw;
    private StatusSignal<AngularVelocity> yawVelocity;

    public Pigeon2IO() {
        pigeon = new Pigeon2(SwerveConstants.kPigeonCANID, "rio");
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.reset();
        pigeon.optimizeBusUtilization();
        
        yaw = pigeon.getYaw();
        yawVelocity = pigeon.getAngularVelocityZWorld();

        // without this line, will get stale CAN frames somehow
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            yaw, yawVelocity
        );

        BaseStatusSignal.setUpdateFrequencyForAll(100, pigeon.getYaw());
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());
    }
}
