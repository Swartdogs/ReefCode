package frc.robot.commands;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class Autos 
{
    private static final AutoFactory autoFactory;

    static
    {
        autoFactory = new AutoFactory(
        Drive.getInstance()::getPose, 
        Drive.getInstance()::setPose, 
        Drive.getInstance()::followTrajectory, 
        true, 
        Drive.getInstance());
    }

    public Command oneCoralAuto(String path)
    {
        return Commands.sequence
        (
            autoFactory.resetOdometry(path),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(path),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output()
        );
    }

    public Command twoCoralAuto(String pathToPegOne, String pathToCS, String pathToPegTwo)
    {
        return Commands.sequence(
            autoFactory.resetOdometry(pathToPegOne),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegOne),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output(),
            autoFactory.trajectoryCmd(pathToCS),
            CompositeCommands.intake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegTwo),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output()
        );
    }

    public Command threeCoralAuto(String pathToPegOne, String pathToCSOne, String pathToPegTwo, String pathToCSTwo, String pathToPegThree)
    {
        return Commands.sequence(
            autoFactory.resetOdometry(pathToPegOne),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegOne),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output(),
            autoFactory.trajectoryCmd(pathToCSOne),
            CompositeCommands.intake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegTwo),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output(),
            autoFactory.trajectoryCmd(pathToCSTwo),
            CompositeCommands.intake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegThree),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output()
        );
    }

    public Command fourCoralAuto(String pathToPegOne, String pathToCSOne, String pathToPegTwo, String pathToCSTwo, String pathToPegThree, String pathToCSThree, String pathToPegFour)
    {
        return Commands.sequence(
            autoFactory.resetOdometry(pathToPegOne),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegOne),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output(),
            autoFactory.trajectoryCmd(pathToCSOne),
            CompositeCommands.intake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegTwo),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output(),
            autoFactory.trajectoryCmd(pathToCSTwo),
            CompositeCommands.intake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegThree),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output(),
            autoFactory.trajectoryCmd(pathToCSThree),
            CompositeCommands.intake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegFour),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.output()
        );
    }
}
