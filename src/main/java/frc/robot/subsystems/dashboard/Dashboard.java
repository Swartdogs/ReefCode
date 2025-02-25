package frc.robot.subsystems.dashboard;

import java.util.LinkedHashMap;
import java.util.Map;
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
import frc.robot.subsystems.manipulator.Manipulator;

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

    public enum DashboardSetting
    {
        FunnelRetractSpeed, FunnelRetractTime, ManipulatorOutputSpeed, ManipulatorIntakeSpeed, ManipulatorL1SpeedMultiplier
    }

    private final DashboardIO                   _io;
    private final DashboardIOInputsAutoLogged   _inputs   = new DashboardIOInputsAutoLogged();
    private final Alert                         _nullAuto;
    private final Map<DashboardSetting, Double> _settings = new LinkedHashMap<DashboardSetting, Double>() {
                                                              {
                                                                  put(DashboardSetting.FunnelRetractSpeed, Constants.Funnel.RETRACT_SPEED);
                                                                  put(DashboardSetting.FunnelRetractTime, Constants.Funnel.DROP_TIME_SECS);
                                                                  put(DashboardSetting.ManipulatorIntakeSpeed, Constants.Manipulator.INTAKE_SPEED);
                                                                  put(DashboardSetting.ManipulatorOutputSpeed, Constants.Manipulator.OUTPUT_SPEED);
                                                                  put(DashboardSetting.ManipulatorL1SpeedMultiplier, Constants.Manipulator.L1_SPEED_MULTIPLIER);
                                                              }
                                                          };

    private Command                             _selectedAuto;

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

        // Robot Values
        _io.setManipulatorLeftMotorOutputPercentSpeed(Manipulator.getInstance().getLeftOutputSpeed());
        _io.setManipulatorRightMotorOutputPercentSpeed(Manipulator.getInstance().getRightOutputSpeed());

        // Update settings
        _settings.put(DashboardSetting.FunnelRetractSpeed, _inputs.funnelRetractPercentSpeed);
        _settings.put(DashboardSetting.FunnelRetractTime, _inputs.funnelRetractTime);
        _settings.put(DashboardSetting.ManipulatorIntakeSpeed, _inputs.manipulatorIntakePercentSpeed);
        _settings.put(DashboardSetting.ManipulatorOutputSpeed, _inputs.manipulatorOutputPercentSpeed);
        _settings.put(DashboardSetting.ManipulatorL1SpeedMultiplier, _inputs.manipulatorL1SpeedMultiplier);

        // Buttons
        if (_inputs.elevatorZeroMinHeightPressed)
        {
            _io.releaseElevatorMinHeightZeroButton();
        }

        if (_inputs.elevatorZeroMaxHeightPressed)
        {
            _io.releaseElevatorMaxHeightZeroButton();
        }

        if (_inputs.elevatorZeroStowHeightPressed)
        {
            _io.releaseElevatorStowHeightZeroButton();
        }

        if (_inputs.elevatorZeroL1HeightPressed)
        {
            _io.releaseElevatorL1HeightZeroButton();
        }

        if (_inputs.elevatorZeroL2HeightPressed)
        {
            _io.releaseElevatorL2HeightZeroButton();
        }

        if (_inputs.elevatorZeroL3HeightPressed)
        {
            _io.releaseElevatorL3HeightZeroButton();
        }

        if (_inputs.elevatorZeroL4HeightPressed)
        {
            _io.releaseElevatorL4HeightZeroButton();
        }

        if (_inputs.elevatorZeroHangHeightPressed)
        {
            _io.releaseElevatorHangHeightZeroButton();
        }

        if (_inputs.driveZeroFLModulePressed)
        {
            _io.releaseDriveFLOffsetZeroButton();
        }

        if (_inputs.driveZeroFRModulePressed)
        {
            _io.releaseDriveFROffsetZeroButton();
        }

        if (_inputs.driveZeroBLModulePressed)
        {
            _io.releaseDriveBLOffsetZeroButton();
        }

        if (_inputs.driveZeroBRModulePressed)
        {
            _io.releaseDriveBROffsetZeroButton();
        }

        if (_inputs.driveZeroModulesPressed)
        {
            _io.releaseDriveModuleOffsetZeroButton();
        }

        if (_inputs.autoStartPosition != null)
        {
            switch (_inputs.autoStartPosition)
            {
                case "Left":
                    _selectedAuto = Constants.Autos.LEFT_1_CORAL_AUTO;
                    break;

                case "Right":
                    if (_inputs.autoNumCoral == 2)
                    {
                        _selectedAuto = Constants.Autos.RIGHT_2_CORAL_AUTO;
                    }
                    else
                    {
                        _selectedAuto = Constants.Autos.RIGHT_1_CORAL_AUTO;
                    }
                    break;

                case "Middle":
                    _selectedAuto = Constants.Autos.MIDDLE_1_CORAL_AUTO;
                    break;

                default:
                    _selectedAuto = null;
            }
        }

        _nullAuto.set(_selectedAuto == null);
    }

    public double getSetting(DashboardSetting setting)
    {
        return _settings.get(setting);
    }

    public Command getSelectedAuto()
    {
        return _selectedAuto;
    }
}
