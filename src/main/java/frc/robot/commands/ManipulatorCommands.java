package frc.robot.commands;

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
        // @formatter:off
        return Commands.sequence
        (
            Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().intake()),
            Commands.waitUntil(() -> Manipulator.getInstance().detectedCoral())
        )
        .finallyDo(() -> Manipulator.getInstance().stop())
        .unless(() -> Manipulator.getInstance().detectedCoral());
        // @formatter:on
    }

    public static Command index()
    {
        // @formatter:off
        return Commands.sequence
        (
            Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().setVolts(-Constants.Manipulator.INDEX_SPEED * Constants.General.MOTOR_VOLTAGE)),
            Commands.waitSeconds(Constants.Manipulator.INDEX_TIME)
        )
        .finallyDo(() -> Manipulator.getInstance().stop());
        // @formatter:on
    }

    public static Command output()
    {
        // @formatter:off
        return Commands.sequence
        (
            Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().output()),
            Commands.waitUntil(() -> !Manipulator.getInstance().isEndSensorTripped())
        )
        .finallyDo(() -> Manipulator.getInstance().stop())
        .unless(() -> !Manipulator.getInstance().isEndSensorTripped());
        // @formatter:on
    }

    public static Command stop()
    {
        return Manipulator.getInstance().runOnce(() -> Manipulator.getInstance().stop());
    }
}
