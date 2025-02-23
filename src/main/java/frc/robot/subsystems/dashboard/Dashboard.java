package frc.robot.subsystems.dashboard;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CompositeCommands;
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

    private final DashboardIO                 _io;
    private final DashboardIOInputsAutoLogged _inputs = new DashboardIOInputsAutoLogged();
    private final Alert                       _nullAuto;

    private Dashboard(DashboardIO io)
    {
        _io = io;

        _nullAuto = new Alert("No Auto Detected", AlertType.kWarning);

        NamedCommands.registerCommand("ExtendToL1", CompositeCommands.setHeight(ElevatorHeight.Level1));
        NamedCommands.registerCommand("ExtendToL2", CompositeCommands.setHeight(ElevatorHeight.Level2));
        NamedCommands.registerCommand("ExtendToL3", CompositeCommands.setHeight(ElevatorHeight.Level3));
        NamedCommands.registerCommand("ExtendToL4", CompositeCommands.setHeight(ElevatorHeight.Level4));
        NamedCommands.registerCommand("Stow", CompositeCommands.setHeight(ElevatorHeight.Stow));
        NamedCommands.registerCommand("Intake", ManipulatorCommands.intake());
        NamedCommands.registerCommand("Output", CompositeCommands.output());
        NamedCommands.registerCommand("Delay", Commands.defer(() -> Commands.waitSeconds(_inputs.autoDelay), Set.of()));
    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Dashboard", _inputs);
    }

    public Command getSelectedAuto()
    {
        var selected = Constants.Lookups.AUTO_LOOKUP.get(_inputs.autoStartPosition + _inputs.autoFirstCoral + _inputs.autoSecondCoral + _inputs.autoThirdCoral);
        _nullAuto.set(selected == null);

        return selected;
    }
}
