package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class LEDIOSim implements LEDIO
{
    private AddressableLED       _led;
    private AddressableLEDBuffer _ledBuffer;
    private AddressableLEDSim    _ledSim;

    public LEDIOSim()
    {
        _led       = new AddressableLED(0);
        _ledBuffer = new AddressableLEDBuffer(Constants.LED.NUM_LEDS);
        _ledSim    = new AddressableLEDSim(_led);

        _led.setLength(Constants.LED.NUM_LEDS);
        _led.setData(_ledBuffer);
        _led.start();
        _ledSim.setRunning(true);
    }

    @Override
    public void setColor(Color color)
    {
        if (color != null)
        {
            for (int i = 0; i < Constants.LED.NUM_LEDS; i++)
            {
                _ledBuffer.setLED(i, color);
            }

            _led.setData(_ledBuffer);
        }
    }
}
