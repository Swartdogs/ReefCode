package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ManipulatorIOSim implements ManipulatorIO
{
    private final DCMotorSim   _leftMotorSim;
    private final DCMotorSim   _rightMotorSim;
    private final DigitalInput _lightSensorStart;
    private final DigitalInput _lightSensorEnd;
    private double             _leftAppliedVolts;
    private double             _rightAppliedVolts;

    public ManipulatorIOSim()
    {
        _leftMotorSim     = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Manipulator.MANIPULATOR_MOTOR, 0.02, Constants.Manipulator.MOTOR_REDUCTION), Constants.Drive.DRIVE_GEARBOX);
        _rightMotorSim    = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Manipulator.MANIPULATOR_MOTOR, 0.02, Constants.Manipulator.MOTOR_REDUCTION), Constants.Drive.DRIVE_GEARBOX);
        _lightSensorEnd   = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_END);
        _lightSensorStart = new DigitalInput(Constants.DIO.MANIPULATOR_LIGHT_SENSOR_START);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs)
    {
        _leftMotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        _rightMotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        inputs.leftCurrentAmps    = Math.abs(_leftMotorSim.getCurrentDrawAmps());
        inputs.rightCurrentAmps   = Math.abs(_rightMotorSim.getCurrentDrawAmps());
        inputs.leftAppliedVolts   = _leftAppliedVolts;
        inputs.rightAppliedVolts  = _rightAppliedVolts;
        inputs.startSensorTripped = _lightSensorStart.get();
        inputs.endSensorTripped   = _lightSensorEnd.get();
    }

    @Override
    public void setLeftVolts(double volts)
    {
        _leftAppliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);

        _leftMotorSim.setInputVoltage(volts);
    }

    @Override
    public void setRightVolts(double volts)
    {
        _rightAppliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);

        _rightMotorSim.setInputVoltage(volts);
    }
}
