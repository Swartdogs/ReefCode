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

    public static Command intake()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().setVolts(Constants.Manipulator.INTAKE_VOLTS)).andThen(Commands.waitUntil(() -> Manipulator.getInstance().hasCoral()))
                .finallyDo(() -> Manipulator.getInstance().setVolts(0)).unless(() -> Manipulator.getInstance().hasCoral());
    }

    public static Command output()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().setVolts(Constants.Manipulator.OUTPUT_VOLTS)).andThen(Commands.waitUntil(() -> !Manipulator.getInstance().hasCoral()))
                .finallyDo(() -> Manipulator.getInstance().setVolts(0)).unless(() -> !Manipulator.getInstance().hasCoral());
    }

    public static Command stop()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().setVolts(0));
    }

    public static Command setVolts(DoubleSupplier voltSupplier)
    {
        return Manipulator.getInstance().run(() -> Manipulator.getInstance().setVolts(voltSupplier.getAsDouble()));
    }
}
