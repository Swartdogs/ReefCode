package frc.robot.subsystems.funnel;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class FunnelIOHardware implements FunnelIO
{
    private final Solenoid _funnelSolenoid;

    public FunnelIOHardware()
    {
        _funnelSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    }

    @Override
    public void updateInputs(FunnelIOInputs inputs)
    {
        inputs.isDropped = _funnelSolenoid.get();
    }

    @Override
    public void setState(boolean state)
    {
        _funnelSolenoid.set(state);
    }
}
