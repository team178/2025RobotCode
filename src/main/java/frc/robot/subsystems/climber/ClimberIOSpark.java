package frc.robot.subsystems.climber;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Velocity;


public class ClimberIOSpark implements ClimberIO {
    private SparkMax motor;
    private RelativeEncoder encoder;

    private double desiredVolts;

    public ClimberIOSpark() {
        motor = new SparkMax(ClimberConstants.kClimberMotorCANID, MotorType.kBrushless);
        motor.setCANTimeout(0);
        encoder = motor.getEncoder();
        
        motor.configure(ClimberConstants.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        desiredVolts = 0;
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.desiredVolts = desiredVolts;
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        inputs.encoderPosition = encoder.getPosition();
    }

    @Override
    public void setClimberVolts(double volts) {
        desiredVolts = volts;
        motor.setVoltage(volts);
    }

    public void setClimberVelocity(Velocity velocity) {
        // motor.
    }
}