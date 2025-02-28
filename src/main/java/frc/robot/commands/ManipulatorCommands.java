package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.manipulator.Manipulator;

public class ManipulatorCommands
{
    private ManipulatorCommands()
    {
    }

    public static Command intake()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().intake()).andThen(Commands.waitUntil(() -> Manipulator.getInstance().detectedCoral())).finallyDo(() -> Manipulator.getInstance().stop())
                .unless(() -> Manipulator.getInstance().detectedCoral());
    }

    public static Command index()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().setVolts(-1)).andThen(Commands.waitSeconds(0.5)).finallyDo(() -> Manipulator.getInstance().stop());
    }

    // public static Command output()
    // {
    // return Manipulator.getInstance().runOnce(() ->
    // Manipulator.getInstance().output()).andThen(Commands.waitUntil(() ->
    // !Manipulator.getInstance().detectedCoral())).finallyDo(() ->
    // Manipulator.getInstance().stop())
    // .unless(() -> !Manipulator.getInstance().detectedCoral());
    // }

    public static Command output()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().output()).andThen(Commands.waitSeconds(1).finallyDo(() -> Manipulator.getInstance().stop()));
    }

    public static Command stop()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().stop());
    }
}
