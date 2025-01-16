package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.Constants;

public class ElevatorCommands
{
    private ElevatorCommands()
    {
    }

    public static Command setHeight(Elevator elevator, double height)
    {
        return Commands.runOnce(() -> elevator.setExtension(height));
    }

    public static Command setHeight(Elevator elevator, ElevatorHeight height)
    {
        return Commands.runOnce(() -> elevator.setExtension(height));
    }

    public static Command setVolts(Elevator elevator, double volts)
    {
        return Commands.runOnce(() -> elevator.setVolts(volts));
    }

    public static Command setVolts(Elevator elevator, DoubleSupplier voltsSupplier)
    {
        return Commands.run(() -> elevator.setVolts(voltsSupplier.getAsDouble()));
    }
}
