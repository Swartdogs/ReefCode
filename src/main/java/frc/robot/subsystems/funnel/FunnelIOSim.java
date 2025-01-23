package frc.robot.subsystems.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class FunnelIOSim implements FunnelIO
{
    private final DCMotorSim _funnelSolenoid;
    private double           _appliedVolts = 0;

    public FunnelIOSim()
    {
        _funnelSolenoid = new DCMotorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.004, 1), DCMotor.getKrakenX60(1));
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs)
    {
        inputs.funnelVolts = _appliedVolts;
    }

    @Override
    public void setVolts(double volts)
    {
        _appliedVolts = MathUtil.clamp(volts, -Constants.General.MOTOR_VOLTAGE, Constants.General.MOTOR_VOLTAGE);
        _funnelSolenoid.setInputVoltage(_appliedVolts);
    }
}
