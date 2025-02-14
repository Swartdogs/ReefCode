package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;

public class CompositeCommands
{
    private CompositeCommands()
    {
    }

    public static Command output(Elevator elevator, Manipulator manipulator)
    {
        return Commands.sequence(ManipulatorCommands.output(manipulator), ElevatorCommands.setHeight(elevator, ElevatorHeight.Stow));
    }
}
