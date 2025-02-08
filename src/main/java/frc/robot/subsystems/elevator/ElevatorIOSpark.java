package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorIOSpark implements ElevatorIO {
    private SparkMax elevatorLeaderMotor;
    private SparkMax elevatorFollowerMotor;
    private SparkMax leftEffectorMotor;
    private SparkMax rightEffectorMotor;

    private SparkAbsoluteEncoder elevatorEncoder;
    private RelativeEncoder leftEffectorEncoder;
    private RelativeEncoder rightEffectorEncoder;

    private SparkClosedLoopController elevatorController;

    private DigitalInput highLimit;
    private DigitalInput lowLimit;
    private DigitalInput upperPhotosensor;
    private DigitalInput lowerPhotosensor;

    public ElevatorIOSpark() {
        elevatorLeaderMotor = new SparkMax(ElevatorConstants.kElevatorLeaderCANID, MotorType.kBrushless);
        elevatorFollowerMotor = new SparkMax(ElevatorConstants.kElevatorFollowerCANID, MotorType.kBrushless);
        leftEffectorMotor = new SparkMax(ElevatorConstants.kLeftEffectorCANID, MotorType.kBrushless);
        rightEffectorMotor = new SparkMax(ElevatorConstants.kRightEffectorCANID, MotorType.kBrushless);

        elevatorLeaderMotor.setCANTimeout(0);
        elevatorFollowerMotor.setCANTimeout(0);
        leftEffectorMotor.setCANTimeout(0);
        rightEffectorMotor.setCANTimeout(0);

        elevatorEncoder = elevatorLeaderMotor.getAbsoluteEncoder();
        leftEffectorEncoder = leftEffectorMotor.getEncoder();
        rightEffectorEncoder = rightEffectorMotor.getEncoder();

        elevatorController = elevatorLeaderMotor.getClosedLoopController();

        leftEffectorMotor.configure(ElevatorConstants.effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightEffectorMotor.configure(ElevatorConstants.effectorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        highLimit = new DigitalInput(ElevatorConstants.kHighLimitDIO);
        lowLimit = new DigitalInput(ElevatorConstants.kLowLimitDIO);
        upperPhotosensor = new DigitalInput(ElevatorConstants.kUpperPhotosensorDIO);
        lowerPhotosensor = new DigitalInput(ElevatorConstants.kLowerPhotosensorDIO);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftEffectorVelocity = leftEffectorEncoder.getVelocity();
        inputs.rightEffectorVelocity = rightEffectorEncoder.getVelocity();
        
        inputs.leftEffectorAppliedVolts = leftEffectorMotor.getAppliedOutput() * leftEffectorMotor.getBusVoltage();
        inputs.rightEffectorAppliedVolts = rightEffectorMotor.getAppliedOutput() * rightEffectorMotor.getBusVoltage();
        inputs.leftEffectorCurrentAmps = leftEffectorMotor.getOutputCurrent();
        inputs.rightEffectorCurrentAmps = rightEffectorMotor.getOutputCurrent();

        inputs.highLimit = highLimit.get();
        inputs.lowLimit = lowLimit.get();
        inputs.upperPhotosensor = upperPhotosensor.get();
        inputs.lowerPhotosensor = lowerPhotosensor.get();
    }

    @Override
    public void setElevatorPosition(ElevatorPosition position) {
        double ffVolts = ElevatorConstants.elevatorControlConstants.kG();
        elevatorController.setReference(
            position.height, ControlType.kPosition,
            (lowLimit.get()) ? ClosedLoopSlot.kSlot1 : ((highLimit.get()) ? ClosedLoopSlot.kSlot2 : ClosedLoopSlot.kSlot0),
            ffVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public void setElevatorOpenLoop(double volts) {
        elevatorLeaderMotor.setVoltage(volts);
    }
    
    @Override
    public void setLeftEffectorVolts(double volts) {
        leftEffectorMotor.setVoltage(volts);
    }
    
    @Override
    public void setRightEffectorVolts(double volts) {
        rightEffectorMotor.setVoltage(volts);
    }
    
    @Override
    public void setEffectorVolts(double left, double right) {
        setLeftEffectorVolts(left);
        setRightEffectorVolts(right);
    }
}
