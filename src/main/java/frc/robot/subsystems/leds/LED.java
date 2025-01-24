package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase
{
    private final LEDIO _io;
    private boolean     _flashing;
    private Color       _color;
    private int         _flashTimer;

    public LED(LEDIO io)
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
            double brightness = 255 / (Constants.LED.FLASH_TIME_SECS / Constants.General.LOOP_PERIOD_SECS / 2);

            if (_flashTimer < Constants.LED.FLASH_TIME_SECS / Constants.General.LOOP_PERIOD_SECS / 2)
            {
                brightness *= _flashTimer;
            }
            else
            {
                brightness *= Constants.LED.FLASH_TIME_SECS / Constants.General.LOOP_PERIOD_SECS - _flashTimer;
            }

            _io.setColor(new Color(_color.red * brightness, _color.green * brightness, _color.blue * brightness));
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
}
