package frc.robot.subsystems.manipulator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import frc.robot.Constants;

public class ManipulatorIOSparkMax implements ManipulatorIO
{
    private final SparkMax _leftMotor;
    private final SparkMax _rightMotor;
    private final DigitalInput _startSensor;
    private final DigitalInput _endSensor;

    public ManipulatorIOSparkMax()
    {
        _leftMotor = new SparkMax(Constants.CAN.MANIPULATOR_LEFT, MotorType.kBrushless);
        _rightMotor = new SparkMax(Constants.CAN.MANIPULATOR_RIGHT, MotorType.kBrushless);
        _startSensor = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_START);
        _endSensor = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_END);

        var leftConfig = new SparkMaxConfig();
        var rightConfig = new SparkMaxConfig();

        leftConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(Constants.General.MOTOR_VOLTAGE);
        rightConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(20).voltageCompensation(Constants.General.MOTOR_VOLTAGE);

        leftConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);
        rightConfig.signals.appliedOutputPeriodMs(20).busVoltagePeriodMs(20).outputCurrentPeriodMs(20);

        _leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        _rightMotor.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs)
    {
        inputs.leftAppliedVolts = _leftMotor.getAppliedOutput() * _leftMotor.getBusVoltage();
        inputs.leftCurrentAmps = _leftMotor.getOutputCurrent();
        inputs.rightAppliedVolts = _rightMotor.getAppliedOutput() * _rightMotor.getBusVoltage();
        inputs.rightCurrentAmps = _rightMotor.getOutputCurrent();

        inputs.startSensorTripped = !_startSensor.get();
        inputs.endSensorTripped = !_endSensor.get();
    }

    @Override
    public void setLeftVolts(double volts)
    {
        _leftMotor.setVoltage(volts);
    }

    @Override
    public void setRightVolts(double volts)
    {
        _rightMotor.setVoltage(volts);
    }
}
