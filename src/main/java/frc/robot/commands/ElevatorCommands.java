package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class ElevatorCommands
{
    private ElevatorCommands()
    {
    }

    public static Command setHeight(Elevator elevator, double height)
    {
        return elevator.runOnce(() -> elevator.setExtension(height));
    }

    public static Command setHeight(Elevator elevator, ElevatorHeight height)
    {
        return elevator.runOnce(() -> elevator.setExtension(height));
    }

    public static Command setVolts(Elevator elevator, double volts)
    {
        return elevator.runOnce(() -> elevator.setVolts(volts));
    }

    public static Command setVolts(Elevator elevator, DoubleSupplier voltsSupplier)
    {
        return elevator.run(() -> elevator.setVolts(voltsSupplier.getAsDouble()));
    }

    public static Command modifyHeight(Elevator elevator, double modification)
    {
        return elevator.runOnce(() -> elevator.modifySetpoint(modification));
    }
}
