package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorCommands
{
    private ManipulatorCommands()
    {
    }

    public static Command intake(Manipulator manipulator)
    {
        return manipulator.runOnce(() -> manipulator.setVolts(Constants.Manipulator.INTAKE_VOLTS)).andThen(Commands.waitUntil(() -> manipulator.hasCoral())).finallyDo(() -> manipulator.setVolts(0)).unless(() -> manipulator.hasCoral());
    }

    public static Command output(Manipulator manipulator)
    {
        return manipulator.runOnce(() -> manipulator.setVolts(Constants.Manipulator.OUTPUT_VOLTS)).andThen(Commands.waitUntil(() -> !manipulator.hasCoral())).finallyDo(() -> manipulator.setVolts(0)).unless(() -> !manipulator.hasCoral());
    }

    public static Command stop(Manipulator manipulator)
    {
        return manipulator.runOnce(() -> manipulator.setVolts(0));
    }

    public static Command setVolts(Manipulator manipulator, DoubleSupplier voltSupplier)
    {
        return manipulator.run(() -> manipulator.setVolts(voltSupplier.getAsDouble()));
    }
}
