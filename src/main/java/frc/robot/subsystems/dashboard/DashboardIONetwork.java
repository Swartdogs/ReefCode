package frc.robot.subsystems.dashboard;

import java.util.ArrayList;
import java.util.List;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DashboardIONetwork implements DashboardIO
{
    private final NetworkTableEntry _matchTimeEntry;
    private final NetworkTableEntry _hasGamePieceEntry;
    private final Field2d           _field;
    private final NetworkTableEntry _elevatorHeightEntry;
    private final NetworkTableEntry _flSwerveAngleEntry;
    private final NetworkTableEntry _frSwerveAngleEntry;
    private final NetworkTableEntry _blSwerveAngleEntry;
    private final NetworkTableEntry _brSwerveAngleEntry;
    private final NetworkTableEntry _manipulatorSpeedEntry;
    private final String            _elevatorMaxHeightKey        = "Elevator Max Height";
    private final String            _elevatorL1HeightKey         = "Elevator L1 Height";
    private final String            _elevatorL2HeightKey         = "Elevator L2 Height";
    private final String            _elevatorL3HeightKey         = "Elevator L3 Height";
    private final String            _elevatorL4HeightKey         = "Elevator L4 Height";
    private final String            _elevatorMinHeightKey        = "Elevator Min Height";
    private final String            _elevatorStowHeightKey       = "Elevator Stow Height";
    private final String            _elevatorHangHeightKey       = "Elevator Hang Height";
    private final String            _flOffsetKey                 = "FL Offset";
    private final String            _frOffsetKey                 = "FR Offset";
    private final String            _blOffsetKey                 = "BL Offset";
    private final String            _brOffsetKey                 = "BR Offset";
    private final String            _funnelVoltageTimeKey        = "Funnel Voltage Time";
    private final String            _manipulatorIntakeVoltageKey = "Manipulator Intake Voltage";
    private final String            _manipulatorOutputVoltageKey = "Manipulator Output Voltage";
    private final String            _elevatorKPKey               = "Elevator KP";
    private final String            _elevatorKIKey               = "Elevator KI";
    private final String            _elevatorKDKey               = "Elevator KD";
    private final String            _elevatorMaxVoltageKey       = "Elevator Max Voltage";

    public DashboardIONetwork()
    {
        _field = new Field2d();
        SmartDashboard.putData(_field);

        _matchTimeEntry        = NetworkTableInstance.getDefault().getEntry("Dashboard/Match Time");
        _hasGamePieceEntry     = NetworkTableInstance.getDefault().getEntry("Dashboard/Has Game Piece");
        _elevatorHeightEntry   = NetworkTableInstance.getDefault().getEntry("Dashboard/Elevator Height");
        _flSwerveAngleEntry    = NetworkTableInstance.getDefault().getEntry("Dashboard/FL Swerve Angle");
        _frSwerveAngleEntry    = NetworkTableInstance.getDefault().getEntry("Dashboard/FR Swerve Angle");
        _blSwerveAngleEntry    = NetworkTableInstance.getDefault().getEntry("Dashboard/BL Swerve Angle");
        _brSwerveAngleEntry    = NetworkTableInstance.getDefault().getEntry("Dashboard/BR Swerve Angle");
        _manipulatorSpeedEntry = NetworkTableInstance.getDefault().getEntry("Dashboard/Manipulator Speed");

        Preferences.initDouble(_elevatorMaxHeightKey, Constants.Elevator.MAX_EXTENSION);
        Preferences.initDouble(_elevatorL1HeightKey, Constants.Elevator.L1_HEIGHT);
        Preferences.initDouble(_elevatorL2HeightKey, Constants.Elevator.L2_HEIGHT);
        Preferences.initDouble(_elevatorL3HeightKey, Constants.Elevator.L3_HEIGHT);
        Preferences.initDouble(_elevatorL4HeightKey, Constants.Elevator.L4_HEIGHT);
        Preferences.initDouble(_elevatorMinHeightKey, 0);
        Preferences.initDouble(_elevatorStowHeightKey, Constants.Elevator.STOW_HEIGHT);
        Preferences.initDouble(_elevatorHangHeightKey, Constants.Elevator.HANG_HEIGHT);
        Preferences.initDouble(_flOffsetKey, Constants.Drive.FL_ZERO_ROTATION.getDegrees());
        Preferences.initDouble(_frOffsetKey, Constants.Drive.FR_ZERO_ROTATION.getDegrees());
        Preferences.initDouble(_blOffsetKey, Constants.Drive.BL_ZERO_ROTATION.getDegrees());
        Preferences.initDouble(_brOffsetKey, Constants.Drive.BL_ZERO_ROTATION.getDegrees());
        Preferences.initDouble(_funnelVoltageTimeKey, Constants.Funnel.DROP_TIME_SECS);
        Preferences.initDouble(_manipulatorIntakeVoltageKey, Constants.Manipulator.INTAKE_VOLTS);
        Preferences.initDouble(_manipulatorOutputVoltageKey, Constants.Manipulator.OUTPUT_VOLTS);
        Preferences.initDouble(_elevatorKPKey, Constants.Elevator.EXTENSION_KP);
        Preferences.initDouble(_elevatorKIKey, Constants.Elevator.EXTENSION_KI);
        Preferences.initDouble(_elevatorKDKey, Constants.Elevator.EXTENSION_KD);
        Preferences.initDouble(_elevatorMaxVoltageKey, Constants.General.MOTOR_VOLTAGE);
    }

    @Override
    public void updateInputs(DashboardIOInputs inputs)
    {
        inputs.elevatorMaxHeight        = Preferences.getDouble(_elevatorMaxHeightKey, Constants.Elevator.MAX_EXTENSION);
        inputs.elevatorL1Height         = Preferences.getDouble(_elevatorL1HeightKey, Constants.Elevator.L1_HEIGHT);
        inputs.elevatorL2Height         = Preferences.getDouble(_elevatorL2HeightKey, Constants.Elevator.L2_HEIGHT);
        inputs.elevatorL3Height         = Preferences.getDouble(_elevatorL3HeightKey, Constants.Elevator.L3_HEIGHT);
        inputs.elevatorL4Height         = Preferences.getDouble(_elevatorL4HeightKey, Constants.Elevator.L4_HEIGHT);
        inputs.elevatorMinHeight        = Preferences.getDouble(_elevatorMinHeightKey, 0);
        inputs.elevatorHangHeight       = Preferences.getDouble(_elevatorHangHeightKey, Constants.Elevator.HANG_HEIGHT);
        inputs.flOffset                 = Rotation2d.fromDegrees(Preferences.getDouble(_flOffsetKey, Constants.Drive.FL_ZERO_ROTATION.getDegrees()));
        inputs.frOffset                 = Rotation2d.fromDegrees(Preferences.getDouble(_frOffsetKey, Constants.Drive.FR_ZERO_ROTATION.getDegrees()));
        inputs.blOffset                 = Rotation2d.fromDegrees(Preferences.getDouble(_blOffsetKey, Constants.Drive.BL_ZERO_ROTATION.getDegrees()));
        inputs.brOffset                 = Rotation2d.fromDegrees(Preferences.getDouble(_brOffsetKey, Constants.Drive.BR_ZERO_ROTATION.getDegrees()));
        inputs.funnelVoltageTime        = Preferences.getDouble(_funnelVoltageTimeKey, Constants.Funnel.DROP_TIME_SECS);
        inputs.manipulatorIntakeVoltage = Preferences.getDouble(_manipulatorIntakeVoltageKey, Constants.Manipulator.INTAKE_VOLTS);
        inputs.manipulatorOutputVoltage = Preferences.getDouble(_manipulatorOutputVoltageKey, Constants.Manipulator.OUTPUT_VOLTS);
        inputs.elevatorKP               = Preferences.getDouble(_elevatorKPKey, Constants.Elevator.EXTENSION_KP);
        inputs.elevatorKI               = Preferences.getDouble(_elevatorKIKey, Constants.Elevator.EXTENSION_KI);
        inputs.elevatorKD               = Preferences.getDouble(_elevatorKDKey, Constants.Elevator.EXTENSION_KD);
        inputs.elevatorMaxVoltage       = Preferences.getDouble(_elevatorMaxVoltageKey, Constants.General.MOTOR_VOLTAGE);

    }

    @Override
    public void setRobotPosition(Pose2d pose)
    {
        _field.setRobotPose(pose);
    }

    @Override
    public void setPath(PathPlannerPath path)
    {
        _field.getObject("path").setPoses(path.getPathPoses());
    }

    @Override
    public void setAuto(List<PathPlannerPath> auto)
    {
        ArrayList<Pose2d> poses = new ArrayList<>();
        for (int i = 0; i < auto.size(); i++)
        {
            poses.addAll(auto.get(i).getPathPoses());
        }
        _field.getObject("auto").setPoses(poses);
    }

    @Override
    public void setMatchTime(double time)
    {
        _matchTimeEntry.setDouble(time);
    }

    @Override
    public void setHasGamePiece(boolean hasGamePiece)
    {
        _hasGamePieceEntry.setBoolean(hasGamePiece);
    }

    @Override
    public void setElevatorHeight(double elevatorHeight)
    {
        _elevatorHeightEntry.setDouble(elevatorHeight);
    }

    @Override
    public void setFLSwerveAngle(Rotation2d flSwerveAngle)
    {
        _flSwerveAngleEntry.setDouble(flSwerveAngle.getDegrees());
    }

    @Override
    public void setFRSwerveAngle(Rotation2d frSwerveAngle)
    {
        _frSwerveAngleEntry.setDouble(frSwerveAngle.getDegrees());
    }

    @Override
    public void setBLSwerveAngle(Rotation2d blSwerveAngle)
    {
        _blSwerveAngleEntry.setDouble(blSwerveAngle.getDegrees());
    }

    @Override
    public void setBRSwerveAngle(Rotation2d brSwerveAngle)
    {
        _brSwerveAngleEntry.setDouble(brSwerveAngle.getDegrees());
    }

    @Override
    public void setManipulatorSpeed(double manipulatorSpeed)
    {
        _manipulatorSpeedEntry.setDouble(manipulatorSpeed);
    }
}
