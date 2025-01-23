package frc.robot.subsystems.funnel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

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
        _appliedVolts = volts;
        _funnelSolenoid.setInputVoltage(volts);
    }
}
