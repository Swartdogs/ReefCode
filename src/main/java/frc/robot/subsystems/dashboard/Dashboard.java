package frc.robot.subsystems.dashboard;

import java.util.List;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;

public class Dashboard extends SubsystemBase
{
    private static Dashboard _instance;

    public static Dashboard getInstance()
    {
        if (_instance == null)
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL, SIM -> new DashboardIONetwork();
                default -> new DashboardIO() {};
            };

            _instance = new Dashboard(io);
        }

        return _instance;
    }

    private final DashboardIO                     _io;
    private final DashboardIOInputsAutoLogged     _inputs = new DashboardIOInputsAutoLogged();
    private final LoggedDashboardChooser<String>  _startingPositionChooser;
    private final LoggedDashboardChooser<String>  _firstCoralChooser;
    private final LoggedDashboardChooser<String>  _secondCoralChooser;
    private final LoggedDashboardChooser<String>  _thirdCoralChooser;
    private final LoggedDashboardChooser<Integer> _autoDelayChooser;
    private final Alert                           _nullAuto;
    private Map<String, PathPlannerAuto>          _autoLookup;
    private Map<String, List<PathPlannerPath>>    _pathLookup;

    private Dashboard(DashboardIO io)
    {
        _io = io;

        var delayChooser = new SendableChooser<Integer>();
        delayChooser.setDefaultOption("0", 0);
        delayChooser.addOption("1", 1);
        delayChooser.addOption("2", 2);
        delayChooser.addOption("3", 3);
        delayChooser.addOption("4", 4);
        delayChooser.addOption("5", 5);
        _autoDelayChooser = new LoggedDashboardChooser<>("Auto Delay", delayChooser);

        var startingPositionChooser = new SendableChooser<String>();
        startingPositionChooser.addOption("Right", "Right");
        startingPositionChooser.addOption("Middle", "Middle");
        startingPositionChooser.addOption("Left", "Left");
        _startingPositionChooser = new LoggedDashboardChooser<>("Starting Position", startingPositionChooser);

        var firstCoralChooser = new SendableChooser<String>();
        firstCoralChooser.addOption("A", "A");
        firstCoralChooser.addOption("B", "B");
        firstCoralChooser.addOption("C", "C");
        firstCoralChooser.addOption("D", "D");
        firstCoralChooser.addOption("E", "E");
        firstCoralChooser.addOption("F", "F");
        firstCoralChooser.addOption("G", "G");
        firstCoralChooser.addOption("H", "H");
        firstCoralChooser.addOption("I", "I");
        firstCoralChooser.addOption("J", "J");
        firstCoralChooser.addOption("K", "K");
        firstCoralChooser.addOption("L", "L");
        _firstCoralChooser = new LoggedDashboardChooser<>("First Coral", firstCoralChooser);

        var secondCoralChooser = new SendableChooser<String>();
        secondCoralChooser.addOption("A", "A");
        secondCoralChooser.addOption("B", "B");
        secondCoralChooser.addOption("C", "C");
        secondCoralChooser.addOption("D", "D");
        secondCoralChooser.addOption("E", "E");
        secondCoralChooser.addOption("F", "F");
        secondCoralChooser.addOption("G", "G");
        secondCoralChooser.addOption("H", "H");
        secondCoralChooser.addOption("I", "I");
        secondCoralChooser.addOption("J", "J");
        secondCoralChooser.addOption("K", "K");
        secondCoralChooser.addOption("L", "L");
        _secondCoralChooser = new LoggedDashboardChooser<>("Second Coral", secondCoralChooser);

        var thirdCoralChooser = new SendableChooser<String>();
        thirdCoralChooser.addOption("A", "A");
        thirdCoralChooser.addOption("B", "B");
        thirdCoralChooser.addOption("C", "C");
        thirdCoralChooser.addOption("D", "D");
        thirdCoralChooser.addOption("E", "E");
        thirdCoralChooser.addOption("F", "F");
        thirdCoralChooser.addOption("G", "G");
        thirdCoralChooser.addOption("H", "H");
        thirdCoralChooser.addOption("I", "I");
        thirdCoralChooser.addOption("J", "J");
        thirdCoralChooser.addOption("K", "K");
        thirdCoralChooser.addOption("L", "L");
        _thirdCoralChooser = new LoggedDashboardChooser<>("Third Coral", thirdCoralChooser);

        _nullAuto = new Alert("No Auto Detected", AlertType.kWarning);

        NamedCommands.registerCommand("ExtendToL1", CompositeCommands.setHeight(ElevatorHeight.Level1));
        NamedCommands.registerCommand("ExtendToL2", CompositeCommands.setHeight(ElevatorHeight.Level2));
        NamedCommands.registerCommand("ExtendToL3", CompositeCommands.setHeight(ElevatorHeight.Level3));
        NamedCommands.registerCommand("ExtendToL4", CompositeCommands.setHeight(ElevatorHeight.Level4));
        NamedCommands.registerCommand("Stow", CompositeCommands.setHeight(ElevatorHeight.Stow));
        NamedCommands.registerCommand("Intake", ManipulatorCommands.intake());
        NamedCommands.registerCommand("Output", CompositeCommands.output());
        NamedCommands.registerCommand("Delay", Commands.defer(() -> Commands.waitSeconds(_autoDelayChooser.get()), Set.of()));

        try
        {
            _autoLookup = Map.of("Right_FC", new PathPlannerAuto("Right_FC"), "Left_JL", new PathPlannerAuto("Left_JL"));
            _pathLookup = Map.of("Right_FC", PathPlannerAuto.getPathGroupFromAutoFile("Right_FC"), "Left_JL", PathPlannerAuto.getPathGroupFromAutoFile("Left_JL"));

        }
        catch (Exception e)
        {
            // TODO: handle exception
        }

    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Dashboard", _inputs);
    }

    public Command getSelectedAuto()
    {
        var selected = Constants.Lookups.AUTO_LOOKUP.get(_startingPositionChooser.get() + _firstCoralChooser.get() + _secondCoralChooser.get() + _thirdCoralChooser.get());
        _nullAuto.set(selected == null);

        return selected;
    }

    public PathPlannerAuto getAuto(String autoName)
    {
        return _autoLookup.get(autoName);
    }

    public List<PathPlannerPath> getPaths(String autoName)
    {
        return _pathLookup.get(autoName);
    }

    public void setRobotPosition(Pose2d pose)
    {
        _io.setRobotPosition(pose);
    }

    public void setPath(PathPlannerPath path)
    {
        _io.setPath(path);
    }

    public void setAuto(List<PathPlannerPath> auto)
    {
        _io.setAuto(auto);
    }
}
