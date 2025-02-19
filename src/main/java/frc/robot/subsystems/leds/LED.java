package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase
{
    private static LED _instance;

    public static LED getInstance()
    {
        if (_instance == null)
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL -> new LEDIOHardware();
                case SIM -> new LEDIOSim();
                default -> new LEDIO() {};
            };

            _instance = new LED(io);
        }

        return _instance;
    }

    private final LEDIO _io;
    private boolean     _flashing;
    private Color       _color;
    private double      _flashTimer;

    private LED(LEDIO io)
    {
        _io = io;
    }

    @Override
    public void periodic()
    {
        if (_flashing)
        {
            _flashTimer++;
            _flashTimer %= Constants.LED.FLASH_TIME_SECS / Constants.General.LOOP_PERIOD_SECS;
            double brightness = 255.0 / (Constants.LED.FLASH_TIME_SECS / Constants.General.LOOP_PERIOD_SECS / 2.0);

            if (_flashTimer < Constants.LED.FLASH_TIME_SECS / Constants.General.LOOP_PERIOD_SECS / 2.0)
            {
                brightness *= _flashTimer;
            }
            else
            {
                brightness *= Constants.LED.FLASH_TIME_SECS / Constants.General.LOOP_PERIOD_SECS - _flashTimer;
            }

            _io.setColor(new Color((int)(_color.red * brightness), (int)(_color.green * brightness), (int)(_color.blue * brightness)));
        }
        else
        {
            _io.setColor(_color);
        }
    }

    public void setColor(Color color)
    {
        _color = color;
    }

    public void setFlashing(boolean flashing)
    {
        _flashing = flashing;
    }

    public void switchDefaultCommand(Command defaultCommand)
    {
        var currentDefaultCommand = getDefaultCommand();

        if (currentDefaultCommand != null)
        {
            currentDefaultCommand.cancel();
        }

        setDefaultCommand(defaultCommand);
    }
}
