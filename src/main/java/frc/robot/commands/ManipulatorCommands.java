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
            // Turn intake on
            Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().intake()),

            // Wait until the end light sensor is triggered
            Commands.waitUntil(() -> Manipulator.getInstance().isEndSensorTripped()),

            // Come back to here (see comments below)
            Commands.repeatingSequence
            (
                // Slow down intake
                Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().slowIntake()),

                // Wait until we detect coral
                Commands.waitUntil(() -> Manipulator.getInstance().detectedCoral()),

                // Stop intake
                Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().stop())
            )
            // Repeat above sequence until we have coral
            .until(() -> Manipulator.getInstance().hasCoral())
        )
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
