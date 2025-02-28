package frc.robot.subsystems.climber;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;

import static edu.wpi.first.units.Units.Volts;

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
    // private SparkClosedLoopController motorPID;

    private boolean prevSet;
    private boolean prevUp;
    private boolean prevDown;
    private boolean isSet;

    private double kG;

    public ClimberIOSpark() {
        climberMotor = new SparkMax(ClimberConstants.kClimberMotorCANID, MotorType.kBrushless);
        climberMotor.setCANTimeout(0);
        encoder = climberMotor.getEncoder();
        pid = climberMotor.getClosedLoopController();
        climberMotor.configure(ClimberConstants.climberSetConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pid.setReference(ClimberConstants.setEncoderValue, ControlType.kPosition);

        prevSet = true;
        prevUp = false;
        prevDown = false;
        isSet = true;

        kG = ClimberConstants.kSetG;
    }
    
    // @Override 
    // public void setClimberVolts(boolean low, boolean high) {
    //     if(low) {
    //         climberMotor.setVoltage(ClimberConstants.climberLowVolts);
    //     }  
    //     else if(high) {
    //         climberMotor.setVoltage(ClimberConstants.climberHighVolts);
    //     }
    //     else{
    //         climberMotor.setVoltage(0);
    //     }
    // }

    @Override
    public void climberInputs(boolean set, boolean up, boolean down) {
        if(set && !prevSet && !up && !down) {
            pid.setReference(ClimberConstants.setEncoderValue, ControlType.kPosition);
            isSet = true;
        }
        if(up && !prevUp) {
            climberMotor.setVoltage(ClimberConstants.upVolts);
        }
        if((!up && prevUp && down) || (down && !prevDown && !up)) {
            climberMotor.setVoltage(ClimberConstants.downVolts);
        }
        if(!up && !down && !isSet) {
            pid.setReference(encoder.getPosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0, kG, ArbFFUnits.kVoltage);
        }
        prevSet = set;
        prevUp = up;
        prevDown = down;
    }

    @Override
    public void updateKG() {
        if(encoder.getPosition() > ClimberConstants.verticalEncoderValue) {
            kG = ClimberConstants.kSetG;
        }
        else if(encoder.getVelocity() > ClimberConstants.maxUpVel) {
            kG = ClimberConstants.kClimbG;
        }
        else if(encoder.getVelocity() < ClimberConstants.maxDownVel) {
            kG = ClimberConstants.kHeheG;
        }
    }

}
