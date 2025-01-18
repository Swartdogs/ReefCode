package frc.robot.subsystems.manipulator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class ManipulatorIOSim implements ManipulatorIO
{
    private final DCMotorSim   _topMotorSim;
    private final DCMotorSim   _bottomMotorSim;
    private double             _appliedVolts;
    private final DigitalInput _lightSensor;

    public ManipulatorIOSim()
    {
        _topMotorSim    = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Manipulator.DRIVE_MOTOR, 0.0, Constants.Manipulator.MOTOR_REDUCTION), Constants.Drive.DRIVE_GEARBOX);
        _bottomMotorSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(Constants.Manipulator.DRIVE_MOTOR, 0.0, Constants.Manipulator.MOTOR_REDUCTION), Constants.Drive.DRIVE_GEARBOX);
        _lightSensor    = new DigitalInput(Constants.DIO.LIGHT_SENSOR);
    }

    @Override
    public void updateInputs(ManipulatorIOInputs inputs)
    {
        _topMotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        _bottomMotorSim.update(Constants.General.LOOP_PERIOD_SECS);
        inputs.topCurrentAmps     = Math.abs(_topMotorSim.getCurrentDrawAmps());
        inputs.bottomCurrentAmps  = Math.abs(_bottomMotorSim.getCurrentDrawAmps());
        inputs.topAppliedVolts    = _appliedVolts;
        inputs.bottomAppliedVolts = _appliedVolts;
        inputs.hasCoral           = !_lightSensor.get();
    }

    @Override
    public void setVolts(double volts)
    {
        _appliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
    }
}
