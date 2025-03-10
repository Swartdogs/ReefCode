package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class DashboardIONetwork implements DashboardIO
{
    // Dashboard Settings
    private final String _elevatorMinHeightKey        = "Dashboard/Dashboard Settings/Elevator Min Height";
    private final String _elevatorMaxHeightKey        = "Dashboard/Dashboard Settings/Elevator Max Height";
    private final String _elevatorStowHeightKey       = "Dashboard/Dashboard Settings/Elevator Stow Height";
    private final String _elevatorL1HeightKey         = "Dashboard/Dashboard Settings/Elevator L1 Height";
    private final String _elevatorL2HeightKey         = "Dashboard/Dashboard Settings/Elevator L2 Height";
    private final String _elevatorL3HeightKey         = "Dashboard/Dashboard Settings/Elevator L3 Height";
    private final String _elevatorL4HeightKey         = "Dashboard/Dashboard Settings/Elevator L4 Height";
    private final String _elevatorHangHeightKey       = "Dashboard/Dashboard Settings/Elevator Hang Height";
    private final String _elevatorHangSpeedKey        = "Dashboard/Dashboard Settings/Elevator Hang Speed";
    private final String _elevatorKPKey               = "Dashboard/Dashboard Settings/Elevator kP";
    private final String _elevatorKDKey               = "Dashboard/Dashboard Settings/Elevator kD";
    private final String _elevatorMaxDownPercentKey   = "Dashboard/Dashboard Settings/Elevator Max Down Speed";
    private final String _elevatorMaxUpPercentKey     = "Dashboard/Dashboard Settings/Elevator Max Up Speed";
    private final String _manipulatorIntakePercentKey = "Dashboard/Dashboard Settings/Manipulator Intake Speed";
    private final String _manipulatorOutputPercentKey     = "Dashboard/Dashboard Settings/Manipulator Output Speed";
    private final String _manipulatorL1SpeedMultiplierKey = "Dashboard/Dashboard Settings/Manipulator L1 Speed Multiplier";
    private final String _funnelRetractPercentKey         = "Dashboard/Dashboard Settings/Funnel Retract Speed";
    private final String _funnelRetractTimeKey            = "Dashboard/Dashboard Settings/Funnel Retract Time";
    private final String _driveFLOffsetKey                = "Dashboard/Dashboard Settings/Drive FL Offset";
    private final String _driveFROffsetKey                = "Dashboard/Dashboard Settings/Drive FR Offset";
    private final String _driveBLOffsetKey                = "Dashboard/Dashboard Settings/Drive BL Offset";
    private final String _driveBROffsetKey                = "Dashboard/Dashboard Settings/Drive BR Offset";

    // Robot Values
    private final NetworkTableEntry _elevatorHeight;
    private final NetworkTableEntry _elevatorSetpoint;
    private final NetworkTableEntry _manipulatorLeftOutputPercent;
    private final NetworkTableEntry _manipulatorRightOutputPercent;
    private final NetworkTableEntry _manipulatorStartSensorTripped;
    private final NetworkTableEntry _manipulatorEndSensorTripped;
    private final NetworkTableEntry _funnelIsDropped;
    private final NetworkTableEntry _matchTime;

    private Rotation2d              _driveFLAngle    = Constants.Drive.FL_ZERO_ROTATION;
    private Rotation2d              _driveFRAngle    = Constants.Drive.FR_ZERO_ROTATION;
    private Rotation2d              _driveBLAngle    = Constants.Drive.BL_ZERO_ROTATION;
    private Rotation2d              _driveBRAngle    = Constants.Drive.BR_ZERO_ROTATION;
    private Rotation2d              _gyroHeading     = new Rotation2d();
    private double                  _driveFLVelocity = 0;
    private double                  _driveFRVelocity = 0;
    private double                  _driveBLVeloticy = 0;
    private double                  _driveBRVelocity = 0;

    // Buttons
    private final NetworkTableEntry _elevatorZeroMinHeightButton;
    private final NetworkTableEntry _elevatorZeroMaxHeightButton;
    private final NetworkTableEntry _elevatorZeroStowHeightButton;
    private final NetworkTableEntry _elevatorZeroL1HeightButton;
    private final NetworkTableEntry _elevatorZeroL2HeightButton;
    private final NetworkTableEntry _elevatorZeroL3HeightButton;
    private final NetworkTableEntry _elevatorZeroL4HeightButton;
    private final NetworkTableEntry _elevatorZeroHangHeightButton;
    private final NetworkTableEntry _driveZeroFLOffsetButton;
    private final NetworkTableEntry _driveZeroFROffsetButton;
    private final NetworkTableEntry _driveZeroBLOffsetButton;
    private final NetworkTableEntry _driveZeroBROffsetButton;
    private final NetworkTableEntry _driveZeroModuleOffsetsButton;

    // Auto Selectors
    private final SendableChooser<Integer> _autoDelayChooser;
    private final SendableChooser<String>  _autoStartPositionChooser;
    private final SendableChooser<Integer> _autoCoralCountChooser;

    // Field
    private final Field2d _field;

    public DashboardIONetwork()
    {
        Preferences.initDouble(_elevatorMinHeightKey, Constants.Elevator.MIN_EXTENSION);
        Preferences.initDouble(_elevatorMaxHeightKey, Constants.Elevator.MAX_EXTENSION);
        Preferences.initDouble(_elevatorStowHeightKey, Constants.Elevator.STOW_HEIGHT);
        Preferences.initDouble(_elevatorL1HeightKey, Constants.Elevator.L1_HEIGHT);
        Preferences.initDouble(_elevatorL2HeightKey, Constants.Elevator.L2_HEIGHT);
        Preferences.initDouble(_elevatorL3HeightKey, Constants.Elevator.L3_HEIGHT);
        Preferences.initDouble(_elevatorL4HeightKey, Constants.Elevator.L4_HEIGHT);
        Preferences.initDouble(_elevatorHangHeightKey, Constants.Elevator.HANG_HEIGHT);
        Preferences.initDouble(_elevatorHangSpeedKey, Constants.Elevator.HANG_SPEED);
        Preferences.initDouble(_elevatorKPKey, Constants.Elevator.EXTENSION_KP);
        Preferences.initDouble(_elevatorKDKey, Constants.Elevator.EXTENSION_KD);
        Preferences.initDouble(_elevatorMaxDownPercentKey, Constants.Elevator.MAX_DOWNWARDS_SPEED);
        Preferences.initDouble(_elevatorMaxUpPercentKey, Constants.Elevator.MAX_UPWARDS_SPEED);
        Preferences.initDouble(_manipulatorIntakePercentKey, Constants.Manipulator.INTAKE_SPEED);
        Preferences.initDouble(_manipulatorOutputPercentKey, Constants.Manipulator.OUTPUT_SPEED);
        Preferences.initDouble(_manipulatorL1SpeedMultiplierKey, Constants.Manipulator.L1_SPEED_MULTIPLIER);
        Preferences.initDouble(_funnelRetractPercentKey, Constants.Funnel.RETRACT_SPEED);
        Preferences.initDouble(_funnelRetractTimeKey, Constants.Funnel.DROP_TIME_SECS);
        Preferences.initDouble(_driveFLOffsetKey, Constants.Drive.FL_ZERO_ROTATION.getDegrees());
        Preferences.initDouble(_driveFROffsetKey, Constants.Drive.FR_ZERO_ROTATION.getDegrees());
        Preferences.initDouble(_driveBLOffsetKey, Constants.Drive.BL_ZERO_ROTATION.getDegrees());
        Preferences.initDouble(_driveBROffsetKey, Constants.Drive.BR_ZERO_ROTATION.getDegrees());

        // Robot Values
        _elevatorHeight                = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Elevator Height");
        _elevatorSetpoint              = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Elevator Setpoint");
        _manipulatorLeftOutputPercent  = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Manipulator Left Percent Output");
        _manipulatorRightOutputPercent = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Manipulator Right Percent Output");
        _manipulatorStartSensorTripped = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Manipulator Start Sensor Tripped");
        _manipulatorEndSensorTripped   = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Manipulator End Sensor Tripped");
        _funnelIsDropped               = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Funnel Is Dropped");
        _matchTime                     = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Match Time");

        _elevatorHeight.setDouble(Constants.Elevator.MIN_EXTENSION);
        _elevatorSetpoint.setDouble(0);

        _manipulatorLeftOutputPercent.setDouble(0);
        _manipulatorRightOutputPercent.setDouble(0);
        _manipulatorStartSensorTripped.setBoolean(false);
        _manipulatorEndSensorTripped.setBoolean(false);
        _matchTime.setDouble(0);

        _funnelIsDropped.setBoolean(false);

        SmartDashboard.putData("Swerve", builder ->
        {
            builder.setSmartDashboardType("SwerveDrive");

            builder.addDoubleProperty("Front Left Angle", () -> _driveFLAngle.getRadians(), null);
            builder.addDoubleProperty("Front Left Velocity", () -> _driveFLVelocity, null);

            builder.addDoubleProperty("Front Right Angle", () -> _driveFRAngle.getRadians(), null);
            builder.addDoubleProperty("Front Right Velocity", () -> _driveFRVelocity, null);

            builder.addDoubleProperty("Back Left Angle", () -> _driveBLAngle.getRadians(), null);
            builder.addDoubleProperty("Back Left Velocity", () -> _driveBLVeloticy, null);

            builder.addDoubleProperty("Back Right Angle", () -> _driveBRAngle.getRadians(), null);
            builder.addDoubleProperty("Back Right Velocity", () -> _driveBRVelocity, null);

            builder.addDoubleProperty("Robot Angle", () -> _gyroHeading.getRadians(), null);
        });

        // Buttons
        _elevatorZeroMinHeightButton  = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Elevator Zero Min Height");
        _elevatorZeroMaxHeightButton  = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Elevator Zero Max Height");
        _elevatorZeroStowHeightButton = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Elevator Zero Stow Height");
        _elevatorZeroL1HeightButton   = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Elevator Zero L1 Height");
        _elevatorZeroL2HeightButton   = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Elevator Zero L2 Height");
        _elevatorZeroL3HeightButton   = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Elevator Zero L3 Height");
        _elevatorZeroL4HeightButton   = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Elevator Zero L4 Height");
        _elevatorZeroHangHeightButton = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Elevator Zero Hang Height");
        _driveZeroFLOffsetButton      = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Drive Zero FL Offset");
        _driveZeroFROffsetButton      = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Drive Zero FR Offset");
        _driveZeroBLOffsetButton      = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Drive Zero BL Offset");
        _driveZeroBROffsetButton      = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Drive Zero BR Offset");
        _driveZeroModuleOffsetsButton = NetworkTableInstance.getDefault().getEntry("Dashboard/Buttons/Drive Zero Module Offsets");

        _elevatorZeroMinHeightButton.setBoolean(false);
        _elevatorZeroMaxHeightButton.setBoolean(false);
        _elevatorZeroStowHeightButton.setBoolean(false);
        _elevatorZeroL1HeightButton.setBoolean(false);
        _elevatorZeroL2HeightButton.setBoolean(false);
        _elevatorZeroL3HeightButton.setBoolean(false);
        _elevatorZeroL4HeightButton.setBoolean(false);
        _elevatorZeroHangHeightButton.setBoolean(false);
        _driveZeroFLOffsetButton.setBoolean(false);
        _driveZeroFROffsetButton.setBoolean(false);
        _driveZeroBLOffsetButton.setBoolean(false);
        _driveZeroBROffsetButton.setBoolean(false);
        _driveZeroModuleOffsetsButton.setBoolean(false);

        // Auto Selectors
        _autoDelayChooser         = new SendableChooser<Integer>();
        _autoStartPositionChooser = new SendableChooser<String>();
        _autoCoralCountChooser    = new SendableChooser<Integer>();

        _autoDelayChooser.setDefaultOption("0", 0);
        _autoDelayChooser.addOption("1", 1);
        _autoDelayChooser.addOption("2", 2);
        _autoDelayChooser.addOption("3", 3);
        _autoDelayChooser.addOption("4", 4);
        _autoDelayChooser.addOption("5", 5);

        _autoStartPositionChooser.addOption("Right", "Right");
        _autoStartPositionChooser.addOption("Middle", "Middle");
        _autoStartPositionChooser.addOption("Left", "Left");

        _autoCoralCountChooser.setDefaultOption("0", 0);
        _autoCoralCountChooser.addOption("1", 1);
        _autoCoralCountChooser.addOption("2", 2);

        // Field
        _field = new Field2d();
        
        // Add sendable elements to NetworkTables
        SmartDashboard.putData("Auto Delay", _autoDelayChooser);
        SmartDashboard.putData("Start Position", _autoStartPositionChooser);
        SmartDashboard.putData("Number of Coral", _autoCoralCountChooser);

        SmartDashboard.putData("Field", _field);
    }

    @Override
    public void updateInputs(DashboardIOInputs inputs)
    {
        // Elevator
        inputs.elevatorMinHeight               = Preferences.getDouble(_elevatorMinHeightKey, Constants.Elevator.MIN_EXTENSION);
        inputs.elevatorMaxHeight               = Preferences.getDouble(_elevatorMaxHeightKey, Constants.Elevator.MAX_EXTENSION);
        inputs.elevatorStowHeight              = Preferences.getDouble(_elevatorStowHeightKey, Constants.Elevator.STOW_HEIGHT);
        inputs.elevatorL1Height                = Preferences.getDouble(_elevatorL1HeightKey, Constants.Elevator.L1_HEIGHT);
        inputs.elevatorL2Height                = Preferences.getDouble(_elevatorL2HeightKey, Constants.Elevator.L2_HEIGHT);
        inputs.elevatorL3Height                = Preferences.getDouble(_elevatorL3HeightKey, Constants.Elevator.L3_HEIGHT);
        inputs.elevatorL4Height                = Preferences.getDouble(_elevatorL4HeightKey, Constants.Elevator.L4_HEIGHT);
        inputs.elevatorHangHeight              = Preferences.getDouble(_elevatorHangHeightKey, Constants.Elevator.HANG_HEIGHT);
        inputs.elevatorKP                      = Preferences.getDouble(_elevatorKPKey, Constants.Elevator.EXTENSION_KP);
        inputs.elevatorKD                      = Preferences.getDouble(_elevatorKDKey, Constants.Elevator.EXTENSION_KD);
        inputs.elevatorMaxDownwardPercentSpeed = Preferences.getDouble(_elevatorMaxDownPercentKey, Constants.Elevator.MAX_DOWNWARDS_SPEED);
        inputs.elevatorMaxUpwardPercentSpeed   = Preferences.getDouble(_elevatorMaxUpPercentKey, Constants.Elevator.MAX_UPWARDS_SPEED);
        inputs.elevatorHangSpeed               = Preferences.getDouble(_elevatorHangSpeedKey, Constants.Elevator.HANG_SPEED);

        // Manipulator
        inputs.manipulatorIntakePercentSpeed = Preferences.getDouble(_manipulatorIntakePercentKey, Constants.Manipulator.INTAKE_SPEED);
        inputs.manipulatorOutputPercentSpeed = Preferences.getDouble(_manipulatorOutputPercentKey, Constants.Manipulator.OUTPUT_SPEED);
        inputs.manipulatorL1SpeedMultiplier  = Preferences.getDouble(_manipulatorL1SpeedMultiplierKey, Constants.Manipulator.L1_SPEED_MULTIPLIER);

        // Funnel
        inputs.funnelRetractPercentSpeed = Preferences.getDouble(_funnelRetractPercentKey, Constants.Funnel.RETRACT_SPEED);
        inputs.funnelRetractTime         = Preferences.getDouble(_funnelRetractTimeKey, Constants.Funnel.DROP_TIME_SECS);

        // Drive
        inputs.driveFLOffset = Rotation2d.fromDegrees(Preferences.getDouble(_driveFLOffsetKey, Constants.Drive.FL_ZERO_ROTATION.getDegrees()));
        inputs.driveFROffset = Rotation2d.fromDegrees(Preferences.getDouble(_driveFROffsetKey, Constants.Drive.FR_ZERO_ROTATION.getDegrees()));
        inputs.driveBLOffset = Rotation2d.fromDegrees(Preferences.getDouble(_driveBLOffsetKey, Constants.Drive.BL_ZERO_ROTATION.getDegrees()));
        inputs.driveBROffset = Rotation2d.fromDegrees(Preferences.getDouble(_driveBROffsetKey, Constants.Drive.BR_ZERO_ROTATION.getDegrees()));

        // Buttons
        inputs.elevatorZeroMinHeightPressed  = _elevatorZeroMinHeightButton.getBoolean(false);
        inputs.elevatorZeroMaxHeightPressed  = _elevatorZeroMaxHeightButton.getBoolean(false);
        inputs.elevatorZeroStowHeightPressed = _elevatorZeroStowHeightButton.getBoolean(false);
        inputs.elevatorZeroL1HeightPressed   = _elevatorZeroL1HeightButton.getBoolean(false);
        inputs.elevatorZeroL2HeightPressed   = _elevatorZeroL2HeightButton.getBoolean(false);
        inputs.elevatorZeroL3HeightPressed   = _elevatorZeroL3HeightButton.getBoolean(false);
        inputs.elevatorZeroL4HeightPressed   = _elevatorZeroL4HeightButton.getBoolean(false);
        inputs.elevatorZeroHangHeightPressed = _elevatorZeroHangHeightButton.getBoolean(false);
        inputs.driveZeroFLModulePressed      = _driveZeroFLOffsetButton.getBoolean(false);
        inputs.driveZeroFRModulePressed      = _driveZeroFROffsetButton.getBoolean(false);
        inputs.driveZeroBLModulePressed      = _driveZeroBLOffsetButton.getBoolean(false);
        inputs.driveZeroBRModulePressed      = _driveZeroBROffsetButton.getBoolean(false);
        inputs.driveZeroModulesPressed       = _driveZeroModuleOffsetsButton.getBoolean(false);

        // Auto Selectors
        inputs.autoDelay         = _autoDelayChooser.getSelected();
        inputs.autoStartPosition = _autoStartPositionChooser.getSelected();
        inputs.autoNumCoral      = _autoCoralCountChooser.getSelected();
    }

    @Override
    public void setRobotPose(Pose2d pose)
    {
        if (pose != null)
        {
            _field.setRobotPose(pose);
        }
    }

    @Override
    public void setElevatorHeight(double height)
    {
        _elevatorHeight.setDouble(height);
    }

    @Override
    public void setElevatorSetpoint(Double setpoint)
    {
        _elevatorSetpoint.setDouble(setpoint == null ? 0 : setpoint);
    }

    @Override
    public void setManipulatorLeftMotorOutputPercentSpeed(double speed)
    {
        _manipulatorLeftOutputPercent.setDouble(speed);
    }

    @Override
    public void setManipulatorRightMotorOutputPercentSpeed(double speed)
    {
        _manipulatorRightOutputPercent.setDouble(speed);
    }

    @Override
    public void setManipulatorStartSensorTripped(boolean tripped)
    {
        _manipulatorStartSensorTripped.setBoolean(tripped);
    }

    @Override
    public void setManipulatorEndSensorTripped(boolean tripped)
    {
        _manipulatorEndSensorTripped.setBoolean(tripped);
    }

    @Override
    public void setFunnelIsDropped(boolean dropped)
    {
        _funnelIsDropped.setBoolean(dropped);
    }

    @Override
    public void setDriveFLAngle(Rotation2d angle)
    {
        _driveFLAngle = angle;
    }

    @Override
    public void setDriveFLVelocity(double velocity)
    {
        _driveFLVelocity = velocity;
    }

    @Override
    public void setDriveFRAngle(Rotation2d angle)
    {
        _driveFRAngle = angle;
    }

    @Override
    public void setDriveFRVelocity(double velocity)
    {
        _driveFRVelocity = velocity;
    }

    @Override
    public void setDriveBLAngle(Rotation2d angle)
    {
        _driveBLAngle = angle;
    }

    @Override
    public void setDriveBLVelocity(double velocity)
    {
        _driveBLVeloticy = velocity;
    }

    @Override
    public void setDriveBRAngle(Rotation2d angle)
    {
        _driveBRAngle = angle;
    }

    @Override
    public void setDriveBRVelocity(double velocity)
    {
        _driveBRVelocity = velocity;
    }

    @Override
    public void setDriveHeading(Rotation2d heading)
    {
        _gyroHeading = heading;
    }

    @Override
    public void setMatchTime(double time)
    {
        _matchTime.setDouble(time);
    }

    @Override
    public void releaseElevatorMinHeightZeroButton()
    {
        _elevatorZeroMinHeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorMaxHeightZeroButton()
    {
        _elevatorZeroMaxHeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorStowHeightZeroButton()
    {
        Preferences.setDouble(_elevatorStowHeightKey, Elevator.getInstance().getExtension());
        _elevatorZeroStowHeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorL1HeightZeroButton()
    {
        Preferences.setDouble(_elevatorL1HeightKey, Elevator.getInstance().getExtension());
        _elevatorZeroL1HeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorL2HeightZeroButton()
    {
        Preferences.setDouble(_elevatorL2HeightKey, Elevator.getInstance().getExtension());
        _elevatorZeroL2HeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorL3HeightZeroButton()
    {
        Preferences.setDouble(_elevatorL3HeightKey, Elevator.getInstance().getExtension());
        _elevatorZeroL3HeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorL4HeightZeroButton()
    {
        Preferences.setDouble(_elevatorL4HeightKey, Elevator.getInstance().getExtension());
        _elevatorZeroL4HeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorHangHeightZeroButton()
    {
        Preferences.setDouble(_elevatorHangHeightKey, Elevator.getInstance().getExtension());
        _elevatorZeroHangHeightButton.setBoolean(false);
    }

    @Override
    public void releaseDriveFLOffsetZeroButton()
    {
        _driveZeroFLOffsetButton.setBoolean(false);
    }

    @Override
    public void releaseDriveFROffsetZeroButton()
    {
        _driveZeroFROffsetButton.setBoolean(false);
    }

    @Override
    public void releaseDriveBLOffsetZeroButton()
    {
        _driveZeroBLOffsetButton.setBoolean(false);
    }

    @Override
    public void releaseDriveBROffsetZeroButton()
    {
        _driveZeroBROffsetButton.setBoolean(false);
    }

    @Override
    public void releaseDriveModuleOffsetZeroButton()
    {
        _driveZeroModuleOffsetsButton.setBoolean(false);
    }
}
