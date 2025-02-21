package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;

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
    private Debouncer                           _debouncer;
    private boolean                             _hasCoral;

    private Manipulator(ManipulatorIO io)
    {
        _io        = io;
        _debouncer = new Debouncer(0.1);
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

    public boolean detectedCoral()
    {
        return (!_inputs.startSensorTripped && _inputs.endSensorTripped) || (Elevator.getInstance().getExtension() > ((Constants.Elevator.L3_HEIGHT + Constants.Elevator.L4_HEIGHT) / 2.0) && _inputs.endSensorTripped);
    }

    public boolean hasCoral()
    {
        return _debouncer.calculate(detectedCoral());
    }

    public boolean isRunning()
    {
        return _inputs.leftAppliedVolts > 0;
    }
}
