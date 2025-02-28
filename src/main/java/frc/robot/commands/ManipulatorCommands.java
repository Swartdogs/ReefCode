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
        // @formatter:off
        return Commands.sequence
        (
            Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().intake()),
            Commands.waitUntil(() -> Manipulator.getInstance().isEndSensorTripped()),
            Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().slowIntake()),
            Commands.waitUntil(() -> Manipulator.getInstance().hasCoral())
        )
        .finallyDo(() -> Manipulator.getInstance().stop())
        .unless(() -> Manipulator.getInstance().hasCoral());
        // @formatter:on
    }

    public static Command intake2()
    {
        // @formatter:off
        return 
            Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().intake()).andThen(
            Commands.waitUntil(() -> Manipulator.getInstance().detectedCoral()))
        .finallyDo(() -> Manipulator.getInstance().stop())
        .unless(() -> Manipulator.getInstance().detectedCoral());
        // @formatter:on
    }

    public static Command intake3()
    {
        // @formatter:off
        return Commands.repeatingSequence(intake2()).until(() -> Manipulator.getInstance().hasCoral())
        .finallyDo(() -> Manipulator.getInstance().stop())
        .unless(() -> Manipulator.getInstance().hasCoral());
        // @formatter:on
    }

    public static Command output()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().output()).andThen(Commands.waitUntil(() -> !Manipulator.getInstance().detectedCoral())).finallyDo(() -> Manipulator.getInstance().stop())
                .unless(() -> !Manipulator.getInstance().detectedCoral());
    }

    public static Command stop()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().stop());
    }
}
