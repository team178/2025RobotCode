package frc.robot.subsystems.climber;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ClimberIOSpark implements ClimberIO {
    private SparkMax climberMotor;
    private RelativeEncoder encoder;
    private SparkClosedLoopController pid;

    private boolean prevUp;
    private boolean prevDown;

    private boolean up;
    private boolean down;

    public SparkMaxConfig climberSetConfig;

    public ClimberIOSpark() {
        climberMotor = new SparkMax(ClimberConstants.kClimberMotorCANID, MotorType.kBrushless);
        climberMotor.setCANTimeout(0);
        encoder = climberMotor.getEncoder();
        pid = climberMotor.getClosedLoopController();
        // climberMotor.configure(ClimberConstants.climberSetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // pid.setReference(ClimberConstants.initialEncoderValue, ControlType.kPosition, ClosedLoopSlot.kSlot0, ClimberConstants.kSetG, ArbFFUnits.kVoltage);

        prevUp = false;
        prevDown = false;

        up = false;
        down = false;

        climberSetConfig = new SparkMaxConfig();
        climberSetConfig
				.idleMode(IdleMode.kBrake)
				.smartCurrentLimit(30)
				.voltageCompensation(12);
			// climberSetConfig.encoder
				// .positionConversionFactor(kEncoderConversionFactor);
			climberSetConfig.closedLoop
				.feedbackSensor((FeedbackSensor.kPrimaryEncoder))
				.p(0)
				.i(0)
				.d(0);
        
        climberMotor.configure(climberSetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void climberInputs(boolean set, boolean up, boolean down) {
        if(set && !up && !down) {
            // isLocked = false;
            pid.setReference(
                ClimberConstants.setEncoderValue,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ClimberConstants.kSetG,
                ArbFFUnits.kVoltage
            );
        }
        if(up && !this.up) {
            // isLocked = false;
            climberMotor.setVoltage(ClimberConstants.upVolts);
        }
        if(!up && down && !this.down) {
            // isLocked = false;
            climberMotor.setVoltage(ClimberConstants.downVolts);
        }
        if(!this.up && !up  && !down && (prevUp || prevDown)) {
            // isLocked = true;
            pid.setReference(
                encoder.getPosition(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ClimberConstants.kClimbG,
                ArbFFUnits.kVoltage
            );
        }
        prevUp = up;
        prevDown = down;
    }

    @Override
    public void update() {
        if(encoder.getPosition() > ClimberConstants.maxPos) {
            up = true;
            pid.setReference(
                encoder.getPosition(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ClimberConstants.kSetG,
                ArbFFUnits.kVoltage
            );
        }

        if(encoder.getPosition() < ClimberConstants.minPos) {
            down = true;
            pid.setReference(
                encoder.getPosition(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ClimberConstants.kClimbG,
                ArbFFUnits.kVoltage
            );
        }
    }

    @Override
    public void setVolts(double volts) {
        climberMotor.setVoltage(volts);
    }

    @Override
    public double getPos() {
        return encoder.getPosition();
    }

    @Override
    public void setReference(double p) {
        climberSetConfig.closedLoop
            .p(p);
        climberMotor.configure(climberSetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pid.setReference(0,ControlType.kPosition, ClosedLoopSlot.kSlot0, 0, ArbFFUnits.kVoltage);
    }
}
