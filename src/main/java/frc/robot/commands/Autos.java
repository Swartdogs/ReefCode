package frc.robot.commands;

import java.util.Set;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.util.Utilities;

public class Autos
{
    public static final AutoFactory autoFactory       = new AutoFactory(Drive.getInstance()::getPose, Drive.getInstance()::setPose, Drive.getInstance()::followTrajectory, false, Drive.getInstance());
    public static final Command     ONE_PIECE_LEFT    = oneCoralAuto("LeftToJ");
    public static final Command     ONE_PIECE_MIDDLE  = oneCoralAuto("MiddleToG");
    public static final Command     ONE_PIECE_RIGHT   = oneCoralAuto("RightToE");
    public static final Command     TWO_PIECE_LEFT    = twoCoralAuto("LeftToK", "KToLeftCS", "LeftCSToL");
    public static final Command     TWO_PIECE_RIGHT   = twoCoralAuto("RightToD", "DToRightCS", "RightCSToC");
    public static final Command     THREE_PIECE_LEFT  = threeCoralAuto("LeftToJ", "JToLeftCS", "LeftCSToK", "KToLeftCS", "LeftCSToL");
    public static final Command     THREE_PIECE_RIGHT = threeCoralAuto("RightToE", "EToRightCS", "RightCSToD", "DToRightCS", "RightCSToC");

    public static Command oneCoralAuto(String path)
    {
        // @formatter:off
        return Commands.sequence
        (
            Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of()),
            autoFactory.resetOdometry(path),
            CompositeCommands.setHeight(ElevatorHeight.Level1),
            autoFactory.trajectoryCmd(path),
            Commands.parallel
            (
                CompositeCommands.autoAlign(Utilities.parseAutoString(path), Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE),
                CompositeCommands.setHeight(ElevatorHeight.Level4)
            ),
            Commands.waitSeconds(1.0),
            CompositeCommands.output()
        );
        // @formatter:on
    }

    public static Command twoCoralAuto(String pathToPegOne, String pathToCS, String pathToPegTwo)
    {
        // @formatter:off
        return Commands.sequence
        (
            Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of()),
            autoFactory.resetOdometry(pathToPegOne),
            CompositeCommands.setHeight(ElevatorHeight.Level1),
            autoFactory.trajectoryCmd(pathToPegOne),
            Commands.parallel
            (
                CompositeCommands.autoAlign(Utilities.parseAutoString(pathToPegOne), Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE),
                CompositeCommands.setHeight(ElevatorHeight.Level4)
            ),
            Commands.waitSeconds(1),
            CompositeCommands.output(),
            autoFactory.trajectoryCmd(pathToCS),
            Drive.getInstance().runOnce(() -> Drive.getInstance().stop()),
            CompositeCommands.intake(),
            CompositeCommands.setHeight(ElevatorHeight.Level1),
            autoFactory.trajectoryCmd(pathToPegTwo),
            Commands.parallel
            (
                CompositeCommands.autoAlign(Utilities.parseAutoString(pathToPegTwo), Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE),
                CompositeCommands.setHeight(ElevatorHeight.Level4)
            ),
            Commands.waitSeconds(1),
            CompositeCommands.output()
        );
        // @formatter:on
    }

    public static Command threeCoralAuto(String pathToPegOne, String pathToCSOne, String pathToPegTwo, String pathToCSTwo, String pathToPegThree)
    {
        // @formatter:off
        return Commands.sequence
        (
            Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of()),
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
        // @formatter:on
    }

    public static Command fourCoralAuto(String pathToPegOne, String pathToCSOne, String pathToPegTwo, String pathToCSTwo, String pathToPegThree, String pathToCSThree, String pathToPegFour)
    {
        // @formatter:off
        return Commands.sequence
        (
            Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of()),
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
        // @formatter:on
    }

    public static Command splitAuto(String path, int numCoral)
    {
        // @formatter:off
        Command auto = Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of());
        for (int i = 0; i < numCoral; i++)
        {
            auto.andThen(
                Commands.parallel
                (
                    CompositeCommands.setHeight(ElevatorHeight.Level1),
                    autoFactory.trajectoryCmd(path, i)
                ),
                Commands.parallel
                (
                    CompositeCommands.snapToBranchAuto(Camera.Front, Utilities.parseAutoString(String.valueOf(path.charAt(path.length() + i - numCoral))), Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE),
                    CompositeCommands.setHeight(ElevatorHeight.Level4)
                ),
                Commands.waitSeconds(0.5),
                CompositeCommands.output(),
                Commands.parallel
                (
                    autoFactory.trajectoryCmd(path, i + 1),
                    Commands.sequence
                    (
                        Commands.waitSeconds(0.5),
                        CompositeCommands.intake()
                    )
                )
            );
        }

        return auto;
        // @formatter:off
    }
}
