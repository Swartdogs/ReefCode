package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.funnel.Funnel;

public class FunnelCommands
{
    private FunnelCommands()
    {
    }

    public static Command drop(Funnel funnel)
    {
        return Commands.startEnd(() -> funnel.setVolts(Constants.Funnel.FUNNEL_VOLTS), () -> funnel.setVolts(0), funnel).withTimeout(Constants.Funnel.DROP_TIME_SECS);
    }
}
