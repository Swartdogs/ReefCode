package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Manipulator extends SubsystemBase
{
    private static Manipulator _instance;

    public static Manipulator getInstance()
    {
        if (_instance == null)
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL -> new ManipulatorIOHardware();
                case SIM -> new ManipulatorIOSim();
                default -> new ManipulatorIO() {};
            };

            _instance = new Manipulator(io);
        }

        return _instance;
    }

    private final ManipulatorIO                 _io;
    private final ManipulatorIOInputsAutoLogged _inputs = new ManipulatorIOInputsAutoLogged();

    private Manipulator(ManipulatorIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Manipulator", _inputs);
    }

    public void setVolts(double volts)
    {
        _io.setVolts(volts);
    }

    public boolean hasCoral()
    {
        return !_inputs.startSensorTripped && _inputs.endSensorTripped;
    }

    public boolean isRunning()
    {
        return _inputs.leftAppliedVolts > 0;
    }
}
