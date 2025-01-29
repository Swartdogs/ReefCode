package frc.robot.subsystems.manipulator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase
{
    private final ManipulatorIO                 _io;
    private final ManipulatorIOInputsAutoLogged _inputs = new ManipulatorIOInputsAutoLogged();

    private final Alert _lightSensorAlert;

    public Manipulator(ManipulatorIO io)
    {
        _io = io;
        _lightSensorAlert = new Alert("Light Sensor Error", AlertType.kError);
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
        return _inputs.hasCoral;
    }
}
