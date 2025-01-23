package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.simulation.SolenoidSim;

public class FunnelIOSim implements FunnelIO
{
    private final SolenoidSim _funnelSolenoid;

    public FunnelIOSim()
    {
        _funnelSolenoid = new SolenoidSim(PneumaticsModuleType.CTREPCM, 0);
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs)
    {
        inputs.isDropped = _funnelSolenoid.getOutput();
    }

    @Override
    public void setState(boolean state)
    {
        _funnelSolenoid.setOutput(state);
    }
}
