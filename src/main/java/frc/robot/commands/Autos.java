package frc.robot.commands;

import java.util.Set;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class Autos
{
    public static final AutoFactory autoFactory       = new AutoFactory(Drive.getInstance()::getPose, Drive.getInstance()::setPose, Drive.getInstance()::followTrajectory, true, Drive.getInstance());
    public static final Command     ONE_PIECE_RIGHT   = oneCoralAuto("RightToE");
    public static final Command     ONE_PIECE_MIDDLE  = oneCoralAuto("MiddleToG");
    public static final Command     ONE_PIECE_LEFT    = oneCoralAuto("LeftToJ");
    public static final Command     TWO_PIECE_RIGHT   = twoCoralAuto("RightToD", "DToRightCS", "RightCSToC");
    public static final Command     TWO_PIECE_LEFT    = twoCoralAuto("LeftToK", "KToLeftCS", "LeftCSToL");
    public static final Command     THREE_PIECE_RIGHT = threeCoralAuto("RightToE", "EToRightCS", "RightCSToD", "DToRightCS", "RightCSToC");
    public static final Command     THREE_PIECE_LEFT  = threeCoralAuto("LeftToJ", "JToLeftCS", "LeftCSToK", "KToLeftCS", "LeftCSToL");

    public static Command oneCoralAuto(String path)
    {
        return Commands.sequence(
                Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of()), autoFactory.resetOdometry(path), CompositeCommands.setHeight(ElevatorHeight.Level3), autoFactory.trajectoryCmd(path),
                CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output()
        );
    }

    public static Command twoCoralAuto(String pathToPegOne, String pathToCS, String pathToPegTwo)
    {
        return Commands.sequence(
                Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of()), autoFactory.resetOdometry(pathToPegOne), CompositeCommands.setHeight(ElevatorHeight.Level3),
                autoFactory.trajectoryCmd(pathToPegOne), CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output(), autoFactory.trajectoryCmd(pathToCS), CompositeCommands.intake(),
                CompositeCommands.setHeight(ElevatorHeight.Level3), autoFactory.trajectoryCmd(pathToPegTwo), CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output()
        );
    }

    public static Command threeCoralAuto(String pathToPegOne, String pathToCSOne, String pathToPegTwo, String pathToCSTwo, String pathToPegThree)
    {
        return Commands.sequence(
                Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of()), autoFactory.resetOdometry(pathToPegOne), CompositeCommands.setHeight(ElevatorHeight.Level3),
                autoFactory.trajectoryCmd(pathToPegOne), CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output(), autoFactory.trajectoryCmd(pathToCSOne), CompositeCommands.intake(),
                CompositeCommands.setHeight(ElevatorHeight.Level3), autoFactory.trajectoryCmd(pathToPegTwo), CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output(), autoFactory.trajectoryCmd(pathToCSTwo),
                CompositeCommands.intake(), CompositeCommands.setHeight(ElevatorHeight.Level3), autoFactory.trajectoryCmd(pathToPegThree), CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output()
        );
    }

    public static Command fourCoralAuto(String pathToPegOne, String pathToCSOne, String pathToPegTwo, String pathToCSTwo, String pathToPegThree, String pathToCSThree, String pathToPegFour)
    {
        return Commands.sequence(
                Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of()), autoFactory.resetOdometry(pathToPegOne), CompositeCommands.setHeight(ElevatorHeight.Level3),
                autoFactory.trajectoryCmd(pathToPegOne), CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output(), autoFactory.trajectoryCmd(pathToCSOne), CompositeCommands.intake(),
                CompositeCommands.setHeight(ElevatorHeight.Level3), autoFactory.trajectoryCmd(pathToPegTwo), CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output(), autoFactory.trajectoryCmd(pathToCSTwo),
                CompositeCommands.intake(), CompositeCommands.setHeight(ElevatorHeight.Level3), autoFactory.trajectoryCmd(pathToPegThree), CompositeCommands.setHeight(ElevatorHeight.Level4), CompositeCommands.output(),
                autoFactory.trajectoryCmd(pathToCSThree), CompositeCommands.intake(), CompositeCommands.setHeight(ElevatorHeight.Level3), autoFactory.trajectoryCmd(pathToPegFour), CompositeCommands.setHeight(ElevatorHeight.Level4),
                CompositeCommands.output()
        );
    }
}
