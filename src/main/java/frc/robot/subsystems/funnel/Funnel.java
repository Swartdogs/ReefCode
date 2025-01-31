package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Funnel extends SubsystemBase
{
    private final FunnelIO                 _io;
    private final FunnelIOInputsAutoLogged _inputs    = new FunnelIOInputsAutoLogged();
    private boolean                        _isDropped = false;

    public Funnel(FunnelIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Funnel", _inputs);
    }

    public void setVolts(double volts)
    {
        _io.setVolts(volts);

        if (volts != 0)
        {
            _isDropped = true;
        }
    }

    @AutoLogOutput(key = "Funnel/Is Dropped")
    public boolean isDropped()
    {
        return _isDropped;
    }
}
