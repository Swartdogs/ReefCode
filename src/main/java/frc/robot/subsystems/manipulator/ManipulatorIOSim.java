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
    private double             _appliedVolts;
    private final DigitalInput _lightSensorStart;
    private final DigitalInput _lightSensorEnd;

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
        inputs.leftAppliedVolts   = _appliedVolts;
        inputs.rightAppliedVolts  = _appliedVolts;
        inputs.startSensorTripped = _lightSensorStart.get();
        inputs.endSensorTripped   = _lightSensorEnd.get();
    }

    @Override
    public void setVolts(double volts)
    {
        _appliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);

        _leftMotorSim.setInputVoltage(volts);
        _rightMotorSim.setInputVoltage(volts);
    }

    @Override
    public void setLeftVolts(double volts)
    {
        _appliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);

        _leftMotorSim.setInputVoltage(volts);
    }
}
