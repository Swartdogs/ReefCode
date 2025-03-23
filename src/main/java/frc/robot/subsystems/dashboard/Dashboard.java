package frc.robot.subsystems.dashboard;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.Autos;
import frc.robot.commands.Autos.AutonomousMode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.util.Utilities;

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
    private Command                           _selectedAuto;

    private Dashboard(DashboardIO io)
    {
        _io = io;

        _nullAuto = new Alert("No Auto Detected", AlertType.kWarning);
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
        _io.setAutoAligned(Drive.getInstance().isAligned());

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
        AutonomousMode mode = null;

        if (_inputs.autoStartPosition != null && _inputs.autoNumCoral > 0)
        {
            mode = switch (_inputs.autoStartPosition)
            {
                case "Left" -> Autos.splitAuto("LeftToJ", _inputs.autoNumCoral);
                case "Middle" -> Autos.splitAuto("MiddleToG", _inputs.autoNumCoral);
                case "Right" -> switch (_inputs.autoNumCoral)
                    {
                        case 2 -> Autos.splitAuto("RightDC", _inputs.autoNumCoral);
                        default -> Autos.splitAuto("RightToE", _inputs.autoNumCoral);
                    };
                default -> null;
            };

            if (mode != null && mode.routine != null)
            {
                _selectedAuto = mode.routine.cmd();
            }
        }
        else
        {
            _selectedAuto = null;
        }

        // Ensure the AprilTag layout is using the correct origin
        Constants.Field.APRIL_TAG_FIELD_LAYOUT.setOrigin(Utilities.isBlueAlliance() ? OriginPosition.kBlueAllianceWallRightSide : OriginPosition.kRedAllianceWallRightSide);

        _nullAuto.set(_selectedAuto == null);

        if (mode != null && mode.trajectories.size() > 0)
        {
            var initialPose = mode.trajectories.get(0).getInitialPose().orElse(new Pose2d());
            var trajectory  = mode.trajectories.stream().flatMap(t -> Arrays.asList(t.getRawTrajectory().getPoses()).stream()).collect(Collectors.toList());

            if (!Utilities.isBlueAlliance())
            {
                var fieldCenter = new Translation2d(Constants.Field.APRIL_TAG_FIELD_LAYOUT.getFieldLength() / 2, Constants.Field.APRIL_TAG_FIELD_LAYOUT.getFieldWidth() / 2);
                var invertAngle = Rotation2d.fromDegrees(180);

                initialPose = initialPose.rotateAround(fieldCenter, invertAngle);
                trajectory = trajectory.stream().map(p -> p.rotateAround(fieldCenter, invertAngle)).toList();
            }

            _io.setRobotPose(initialPose);
            _io.setTrajectory(trajectory);
        }
        else
        {
            _io.setRobotPose(new Pose2d());
            _io.setTrajectory(List.of());
        }
    }

    public double getAutoDelay()
    {
        return _inputs.autoDelay;
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

    public double getManipulatorCoralIntakePercentSpeed()
    {

        return _inputs.manipulatorCoralIntakePercentSpeed;
    }

    public double getManipulatorCoralOutputPercentSpeed()
    {
        return _inputs.manipulatorCoralOutputPercentSpeed;
    }

    public double getManipulatorAlgaeIntakePercentSpeed()
    {

        return _inputs.manipulatorAlgaeIntakePercentSpeed;
    }

    public double getManipulatorAlgaeOutputPercentSpeed()
    {
        return _inputs.manipulatorAlgaeOutputPercentSpeed;
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

    public double getElevatorLowAlgaeHeight()
    {
        return _inputs.elevatorLowAlgaeHeight;
    }

    public double getElevatorHighAlgaeHeight()
    {
        return _inputs.elevatorHighAlgaeHeight;
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
