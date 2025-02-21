package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class ElevatorCommands
{
    private ElevatorCommands()
    {
    }

    public static Command setHeight(double height)
    {
        return Elevator.getInstance().runOnce(() -> Elevator.getInstance().setExtension(height));
    }

    public static Command setHeight(ElevatorHeight height)
    {
        if (height == ElevatorHeight.Stow)
        {
            return stow();
        }
        else
        {
            return Elevator.getInstance().runOnce(() -> Elevator.getInstance().setExtension(height));
        }
    }

    public static Command stow()
    {
        return Commands.sequence(
                Elevator.getInstance().runOnce(() -> Elevator.getInstance().setExtension(ElevatorHeight.Stow)), Commands.waitUntil(() -> Elevator.getInstance().atSetpoint()),
                Elevator.getInstance().runOnce(() -> Elevator.getInstance().stop())
        );
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
        return Elevator.getInstance().runOnce(() -> Elevator.getInstance().setVolts(Constants.Elevator.HANG_VOLTAGE));
    }
}
