package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LED;

public class LEDCommands
{
    private LEDCommands()
    {

    }

    public static Command setColor(LED led, Color color)
    {
        return led.runOnce(() -> led.setColor(color));
    }

    public static Command flashColor(LED led, Color color)
    {
        return led.run(() ->
        {
            led.setFlashing(true);
            led.setColor(color);
        }).finallyDo(() -> led.setFlashing(false));
    }

    public static Command setDefaultColor(LED led, Color color)
    {
        return led.runOnce(() -> led.switchDefaultCommand(setColor(led, color)));
    }
}
