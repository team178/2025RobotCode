package frc.robot.subsystems.climber;

import frc.robot.Constants.ClimberConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class ClimberIOSpark implements ClimberIO {
    private SparkMax climberMotor;
    
    public ClimberIOSpark() {
        climberMotor = new SparkMax(ClimberConstants.kClimberMotorCANID, MotorType.kBrushless);
        climberMotor.setCANTimeout(0);

        climberMotor.configure(ClimberConstants.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    public void setClimberVolts(boolean low, boolean high) {
        if(low) {
            climberMotor.setVoltage(ClimberConstants.climberLowVolts);
        }  
        else if(high) {
            climberMotor.setVoltage(ClimberConstants.climberHighVolts);
        }
        else{
            climberMotor.setVoltage(0);
        }
    }

}
