package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.leds.LED;

public class LEDCommands
{
    private LEDCommands()
    {

    }

    public static Command setColor(Color color)
    {
        return LED.getInstance().runOnce(() -> LED.getInstance().setColor(color));
    }

    public static Command flashColor(Color color)
    {
        return LED.getInstance().runOnce(() ->
        {
            LED.getInstance().setFlashing(true);
            LED.getInstance().setColor(color);
        }).andThen(Commands.idle(LED.getInstance())).finallyDo(() -> LED.getInstance().setFlashing(false));
    }

    public static Command setDefaultColor(Color color)
    {
        return LED.getInstance().runOnce(() -> LED.getInstance().switchDefaultCommand(setColor(color)));
    }
}
