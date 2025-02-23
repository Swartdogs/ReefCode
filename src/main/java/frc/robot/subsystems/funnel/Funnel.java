package frc.robot.subsystems.funnel;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Funnel extends SubsystemBase
{
    private static Funnel _instance;

    public static Funnel getInstance()
    {
        if (_instance == null)
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL -> new FunnelIOHardware();
                case SIM -> new FunnelIOSim();
                default -> new FunnelIO() {};
            };

            _instance = new Funnel(io);
        }

        return _instance;
    }

    private final FunnelIO                 _io;
    private final FunnelIOInputsAutoLogged _inputs             = new FunnelIOInputsAutoLogged();
    private final Alert                    _funnelDroppedAlert = new Alert("The Funnel has been dropped", AlertType.kInfo);
    private boolean                        _isDropped          = false;

    private Funnel(FunnelIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Funnel", _inputs);

        _funnelDroppedAlert.set(_isDropped);

        Logger.recordOutput("Funnel/Is Dropped", isDropped());
    }

    public void setVolts(double volts)
    {
        _io.setVolts(volts);

        if (volts != 0)
        {
            _isDropped = true;
        }
    }

    public boolean isDropped()
    {
        return _isDropped;
    }
}
