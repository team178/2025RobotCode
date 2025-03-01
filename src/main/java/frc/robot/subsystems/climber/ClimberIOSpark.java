package frc.robot.subsystems.climber;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

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

    private boolean prevSet;
    private boolean prevUp;
    private boolean prevDown;

    private boolean up;
    private boolean down;

    private double kG;
    private boolean isLocked;

    public ClimberIOSpark() {
        climberMotor = new SparkMax(ClimberConstants.kClimberMotorCANID, MotorType.kBrushless);
        climberMotor.setCANTimeout(0);
        encoder = climberMotor.getEncoder();
        pid = climberMotor.getClosedLoopController();
        climberMotor.configure(ClimberConstants.climberSetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pid.setReference(ClimberConstants.initialEncoderValue, ControlType.kPosition, ClosedLoopSlot.kSlot0, ClimberConstants.kSetG, ArbFFUnits.kVoltage);

        prevSet = true;
        prevUp = false;
        prevDown = false;

        up = false;
        down = false;

        kG = ClimberConstants.kClimbG;
        isLocked = true;
    }

    @Override
    public void climberInputs(boolean set, boolean up, boolean down) {
        if(set && !prevSet && !up && !down) {
            isLocked = false;
            pid.setReference(
                ClimberConstants.setEncoderValue,
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                ClimberConstants.kSetG,
                ArbFFUnits.kVoltage
            );
        }
        if(up && !this.up) {
            isLocked = false;
            climberMotor.setVoltage(ClimberConstants.upVolts);
        }
        if(!up && down && !this.down) {
            isLocked = false;
            climberMotor.setVoltage(ClimberConstants.downVolts);
        }
        if(!this.up && ((!up && prevUp) || (!up && !down && prevDown))) {
            isLocked = true;
            pid.setReference(
                encoder.getPosition(),
                ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                kG,
                ArbFFUnits.kVoltage
            );
        }
        prevSet = set;
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
                kG,
                ArbFFUnits.kVoltage
            );
        }

        if(isLocked && (encoder.getVelocity() > ClimberConstants.maxVel)) {
            kG = ClimberConstants.kClimbG;
        }
        
        if(isLocked && (encoder.getVelocity() < ClimberConstants.minVel)) {
            kG = ClimberConstants.kLightG;
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
}
