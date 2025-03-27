package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.util.Utilities;

public class Autos
{
    public static final AutoFactory    autoFactory      = new AutoFactory(Drive.getInstance()::getPose, Drive.getInstance()::setPose, Drive.getInstance()::followTrajectory, false, Drive.getInstance());
    public static final AutonomousMode ONE_PIECE_LEFT   = splitAuto("LeftToJ", 1);
    public static final AutonomousMode ONE_PIECE_MIDDLE = splitAuto("MiddleToG", 1);
    public static final AutonomousMode ONE_PIECE_RIGHT  = splitAuto("RightToE", 1);
    public static final AutonomousMode TWO_PIECE_RIGHT  = splitAuto("RightEC", 2);
    // public static final Command ONE_PIECE_LEFT = oneCoralAuto("LeftToJ");
    // public static final Command ONE_PIECE_MIDDLE = oneCoralAuto("MiddleToG");
    // public static final Command ONE_PIECE_RIGHT = oneCoralAuto("RightToE");
    // public static final Command TWO_PIECE_LEFT = twoCoralAuto("LeftToK",
    // "KToLeftCS", "LeftCSToL");
    // public static final Command TWO_PIECE_RIGHT = twoCoralAuto("RightToD",
    // "DToRightCS", "RightCSToC");
    // public static final Command THREE_PIECE_LEFT = threeCoralAuto("LeftToJ",
    // "JToLeftCS", "LeftCSToK", "KToLeftCS", "LeftCSToL");
    // public static final Command THREE_PIECE_RIGHT = threeCoralAuto("RightToE",
    // "EToRightCS", "RightCSToD", "DToRightCS", "RightCSToC");

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
            CompositeCommands.coralOutput()
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
            ManipulatorCommands.coralOutput(),
            Commands.parallel
            (
                autoFactory.trajectoryCmd(pathToCS),
                CompositeCommands.setHeight(ElevatorHeight.Stow)   
            ),
            Drive.getInstance().runOnce(() -> Drive.getInstance().stop()),
            CompositeCommands.coralIntake(),
            CompositeCommands.setHeight(ElevatorHeight.Level1),
            autoFactory.trajectoryCmd(pathToPegTwo),
            Commands.parallel
            (
                CompositeCommands.autoAlign(Utilities.parseAutoString(pathToPegTwo), Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE),
                CompositeCommands.setHeight(ElevatorHeight.Level4)
            ),
            Commands.waitSeconds(1),
            CompositeCommands.coralOutput()
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
            CompositeCommands.coralOutput(),
            autoFactory.trajectoryCmd(pathToCSOne),
            CompositeCommands.coralIntake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegTwo),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.coralOutput(),
            autoFactory.trajectoryCmd(pathToCSTwo),
            CompositeCommands.coralIntake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegThree),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.coralOutput()
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
            CompositeCommands.coralOutput(),
            autoFactory.trajectoryCmd(pathToCSOne),
            CompositeCommands.coralIntake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegTwo),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.coralOutput(),
            autoFactory.trajectoryCmd(pathToCSTwo),
            CompositeCommands.coralIntake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegThree),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.coralOutput(),
            autoFactory.trajectoryCmd(pathToCSThree),
            CompositeCommands.coralIntake(),
            CompositeCommands.setHeight(ElevatorHeight.Level3),
            autoFactory.trajectoryCmd(pathToPegFour),
            CompositeCommands.setHeight(ElevatorHeight.Level4),
            CompositeCommands.coralOutput()
        );
        // @formatter:on
    }

    public static AutonomousMode splitAuto(String path, int numCoral)
    {
        // @formatter:off
        AutoRoutine routine = autoFactory.newRoutine("Autonomous");
        ArrayList<AutoTrajectory> trajectories = new ArrayList<>();

        Command auto = Commands.defer(() -> Commands.waitSeconds(Dashboard.getInstance().getAutoDelay()), Set.of());

        for (int i = 0; i < numCoral; i++)
        {
            var traj1 = routine.trajectory(path, 2 * i);
            var traj2 = routine.trajectory(path, 2 * i + 1);

            trajectories.add(traj1);
            trajectories.add(traj2);

            if (i == 0)
            {
                auto = auto.andThen(traj1.resetOdometry());
            }

            auto = auto.andThen(
                Commands.parallel
                (
                    CompositeCommands.setHeight(ElevatorHeight.Coast),
                    traj1.cmd()
                ),
                Commands.parallel
                (
                    //CompositeCommands.autoAlign(Utilities.parseAutoString(String.valueOf(path.charAt(path.length() + i - numCoral))), Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE),
                    DriveCommands.stop(),
                    CompositeCommands.setHeight(ElevatorHeight.Level4)
                ),
                Commands.waitSeconds(0.5),
                ManipulatorCommands.coralOutput(),
                Commands.parallel
                (
                    Commands.sequence
                    (
                        Commands.parallel
                        (
                            traj2.cmd(),
                            ElevatorCommands.setHeight(ElevatorHeight.Stow)
                        ),
                        DriveCommands.stop()
                    ),
                    Commands.sequence
                    (
                        Commands.waitSeconds(0.5),
                        CompositeCommands.coralIntake()
                    )
                )
            );
        }

        routine.active().onTrue(auto);

        return new AutonomousMode(routine, trajectories);
        // @formatter:off
    }

    public static class AutonomousMode
    {
        public final AutoRoutine routine;
        public final List<AutoTrajectory> trajectories;

        public AutonomousMode(AutoRoutine routine, List<AutoTrajectory> trajectories)
        {
            this.routine = routine;
            this.trajectories = trajectories;
        }
    }
}
