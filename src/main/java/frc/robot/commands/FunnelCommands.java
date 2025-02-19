package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.funnel.Funnel;

public class FunnelCommands
{
    private FunnelCommands()
    {
    }

    public static Command drop()
    {
        return Commands.startEnd(() -> Funnel.getInstance().setVolts(Constants.Funnel.FUNNEL_VOLTS), () -> Funnel.getInstance().setVolts(0), Funnel.getInstance()).withTimeout(Constants.Funnel.DROP_TIME_SECS);
    }

    public static Command setVolts(DoubleSupplier voltsSupplier)
    {
        return Funnel.getInstance().run(() -> Funnel.getInstance().setVolts(voltsSupplier.getAsDouble()));
    }
}
