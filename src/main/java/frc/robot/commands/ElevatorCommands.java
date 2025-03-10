package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class ElevatorCommands
{
    private ElevatorCommands()
    {
    }

    public static Command setHeight(ElevatorHeight height)
    {
        Command command;

        if (height == ElevatorHeight.Stow)
        {
            command = stow();
        }
        else
        {
            command = Elevator.getInstance().runOnce(() -> Elevator.getInstance().setExtension(height));
        }

        return command;
    }

    public static Command stow()
    {
        // @formatter:off
        return Commands.sequence
        (
            Elevator.getInstance().runOnce(() -> Elevator.getInstance().setExtension(ElevatorHeight.Stow)),
            Commands.waitUntil(() -> Elevator.getInstance().atSetpoint()),
            Elevator.getInstance().runOnce(() -> Elevator.getInstance().stop())
        );
        // @formatter:on
    }

    public static Command setVolts(double volts)
    {
        return Elevator.getInstance().runOnce(() -> Elevator.getInstance().setVolts(volts));
    }

    public static Command setVolts(DoubleSupplier voltsSupplier)
    {
        return Elevator.getInstance().run(() -> Elevator.getInstance().setVolts(voltsSupplier.getAsDouble()));
    }

    public static Command modifyHeight(double modification)
    {
        return Elevator.getInstance().runOnce(() -> Elevator.getInstance().modifySetpoint(modification));
    }

    public static Command hangExecute()
    {
        // @formatter:off
        return Elevator.getInstance().run(() -> Elevator.getInstance().setVolts(-Dashboard.getInstance().getElevatorHangSpeed() * Constants.General.MOTOR_VOLTAGE))
            .finallyDo(() -> Elevator.getInstance().stop());
        // @formatter:on
    }
}
