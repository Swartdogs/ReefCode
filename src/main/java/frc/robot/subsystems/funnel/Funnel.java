package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase
{
    private final FunnelIO                 _io;
    private final FunnelIOInputsAutoLogged _inputs = new FunnelIOInputsAutoLogged();

    public Funnel(FunnelIO io)
    {
        _io = io;
    }

    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Funnel", _inputs);
    }

    public void set(boolean state)
    {
        _io.setState(state);
    }
}
