package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.swerve.SDSModuleIO.SDSModuleIOInputs;

public class SDSSwerveModule {
    public static final NetworkTable constantPreferences = NetworkTableInstance.getDefault().getTable("Swerve Modules");
    
    private String name;
    private int index;
    private NetworkTable moduleNT;

    private SDSModuleIO io;
    private SDSModuleIOInputsAutoLogged inputs = new SDSModuleIOInputsAutoLogged();
    private SwerveModuleState desiredModuleState;

    public SDSSwerveModule(String name, SDSModuleIO io, int index) {
        Preferences.initBoolean("Swerve Modules/" + name + "enabled", true);
        this.name = name;
        this.index = index;
        moduleNT = constantPreferences.getSubTable(name);

        this.io = io;
        desiredModuleState = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        setDesiredState(desiredState, true);
    }

    public void setDesiredState(SwerveModuleState state, boolean optimize) {
        if(!enabled()) {
            stopDrive();
            return;
        }
        if(optimize) {
            state.optimize(inputs.turnPosition);
            // state.cosineScale(inputs.turnPosition);
        }
        desiredModuleState = state;
        io.setTurnPosition(state.angle);
        io.setDriveVelocityRadPerSec(state.speedMetersPerSecond / SwerveConstants.kWheelRadiusMeters);
    }

    public SwerveModuleState stopState() {
        return new SwerveModuleState(0, desiredModuleState.angle);
    }

    public void openLoop(double turn, double drive) {
        io.setTurnOpenLoop(turn);
        io.setDriveOpenLoop(drive);
    }

    public void stopDrive() {
        io.setTurnOpenLoop(0);
        io.setDriveOpenLoop(0);
    }

    public SwerveModuleState getCurrentState() {
        return new SwerveModuleState(
            inputs.driveVelocityRadPerSec * SwerveConstants.kWheelRadiusMeters,
            inputs.turnPosition
        );
    }

    public SwerveModuleState getDesiredModuleState() {
        return desiredModuleState;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            inputs.drivePositionRad * SwerveConstants.kWheelRadiusMeters,
            inputs.turnPosition
        );
    }

    public void updateControlConstants() { // don't spam run
        io.updateControlConstants();
    }

    public SDSModuleIOInputs getInputs() {
        return inputs;
    }

    public boolean enabled() {
        return Preferences.getBoolean("Swerve Modules/" + name + "enabled", true);
    }
    
    public void putInfo() {
        moduleNT.getEntry("desiredspeed").setDouble(desiredModuleState.speedMetersPerSecond);
        moduleNT.getEntry("desiredangle").setDouble(desiredModuleState.angle.getRadians());
    }

    public void periodic() {
        putInfo();
        io.updateInputs(inputs);
        Logger.processInputs("Swerve/Module" + index, inputs);
    }
}
