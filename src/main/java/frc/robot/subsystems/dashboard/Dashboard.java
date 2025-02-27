package frc.robot.subsystems.dashboard;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.funnel.Funnel;
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

    private final DashboardIO                 _io;
    private final DashboardIOInputsAutoLogged _inputs = new DashboardIOInputsAutoLogged();
    private final Alert                       _nullAuto;
    private PathPlannerAuto                   _selectedAuto;

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
        var states = Drive.getInstance().getModuleStates();
        _io.setDriveFLAngle(states[0].angle);
        _io.setDriveFLVelocity(states[0].speedMetersPerSecond);
        _io.setDriveFRAngle(states[1].angle);
        _io.setDriveFRVelocity(states[1].speedMetersPerSecond);
        _io.setDriveBLAngle(states[2].angle);
        _io.setDriveBLVelocity(states[2].speedMetersPerSecond);
        _io.setDriveBRAngle(states[3].angle);
        _io.setDriveBRVelocity(states[3].speedMetersPerSecond);
        _io.setDriveHeading(Drive.getInstance().getRotation());

        _io.setManipulatorLeftMotorOutputPercentSpeed(Manipulator.getInstance().getLeftOutputSpeed());
        _io.setManipulatorRightMotorOutputPercentSpeed(Manipulator.getInstance().getRightOutputSpeed());
        _io.setManipulatorStartSensorTripped(Manipulator.getInstance().isStartSensorTripped());
        _io.setManipulatorEndSensorTripped(Manipulator.getInstance().isEndSensorTripped());

        _io.setFunnelIsDropped(Funnel.getInstance().isDropped());

        _io.setElevatorHeight(Elevator.getInstance().getExtension());
        _io.setElevatorSetpoint(Elevator.getInstance().getSetpoint());

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

        // Match Time
        _io.setMatchTime(DriverStation.getMatchTime());

        // Autonomous
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

        if (_selectedAuto != null)
        {
            _io.setRobotPose(_selectedAuto.getStartingPose());
        }
        else
        {
            _io.setRobotPose(null);
        }

        _nullAuto.set(_selectedAuto == null);
    }

    public Command getSelectedAuto()
    {
        return _selectedAuto;
    }

    public double getFunnelRetractPercentSpeed()
    {
        return _inputs.funnelRetractPercentSpeed;
    }

    public double getFunnelRetractTime()
    {
        return _inputs.funnelRetractTime;
    }

    public double getManipulatorIntakePercentSpeed()

    {

        return _inputs.manipulatorIntakePercentSpeed;
    }

    public double getManipulatorOutputPercentSpeed()
    {
        return _inputs.manipulatorOutputPercentSpeed;
    }

    public double getManipulatorL1SpeedMultiplier()
    {
        return _inputs.manipulatorL1SpeedMultiplier;
    }

    public double getElevatorMinHeight()
    {
        return _inputs.elevatorMinHeight;
    }

    public double getElevatorMaxHeight()
    {
        return _inputs.elevatorMaxHeight;
    }

    public double getElevatorStowHeight()
    {
        return _inputs.elevatorStowHeight;
    }

    public double getElevatorL1Height()
    {
        return _inputs.elevatorL1Height;
    }

    public double getElevatorL2Height()
    {
        return _inputs.elevatorL2Height;
    }

    public double getElevatorL3Height()
    {
        return _inputs.elevatorL3Height;
    }

    public double getElevatorL4Height()
    {
        return _inputs.elevatorL4Height;
    }

    public double getElevatorHangHeight()
    {
        return _inputs.elevatorHangHeight;
    }

    public double getElevatorHangSpeed()
    {
        return _inputs.elevatorHangSpeed;
    }

    public double getElevatorKP()
    {
        return _inputs.elevatorKP;
    }

    public double getElevatorKD()
    {
        return _inputs.elevatorKD;
    }

    public double getElevatorMaxDownwardPercentSpeed()
    {
        return _inputs.elevatorMaxDownwardPercentSpeed;
    }

    public double getElevatorMaxUpwardPercentSpeed()
    {
        return _inputs.elevatorMaxUpwardPercentSpeed;
    }
}
