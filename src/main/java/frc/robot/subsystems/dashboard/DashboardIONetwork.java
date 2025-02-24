package frc.robot.subsystems.dashboard;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class DashboardIONetwork implements DashboardIO
{
    // Dashboard Settings
    private final String _elevatorMinHeightKey            = "Dashboard/Dashboard Settings/Elevator Min Height";
    private final String _elevatorMaxHeightKey            = "Dashboard/Dashboard Settings/Elevator Max Height";
    private final String _elevatorStowHeightKey           = "Dashboard/Dashboard Settings/Elevator Stow Height";
    private final String _elevatorL1HeightKey             = "Dashboard/Dashboard Settings/Elevator L1 Height";
    private final String _elevatorL2HeightKey             = "Dashboard/Dashboard Settings/Elevator L2 Height";
    private final String _elevatorL3HeightKey             = "Dashboard/Dashboard Settings/Elevator L3 Height";
    private final String _elevatorL4HeightKey             = "Dashboard/Dashboard Settings/Elevator L4 Height";
    private final String _elevatorHangHeightKey           = "Dashboard/Dashboard Settings/Elevator Hang Height";
    private final String _elevatorHangSpeedKey            = "Dashboard/Dashboard Settings/Elevator Hang Speed";
    private final String _elevatorKPKey                   = "Dashboard/Dashboard Settings/Elevator kP";
    private final String _elevatorKDKey                   = "Dashboard/Dashboard Settings/Elevator kD";
    private final String _elevatorMaxDownPercentKey       = "Dashboard/Dashboard Settings/Elevator Max Down Speed";
    private final String _elevatorMaxUpPercentKey         = "Dashboard/Dashboard Settings/Elevator Max Up Speed";
    private final String _manipulatorIntakePercentKey     = "Dashboard/Dashboard Settings/Manipulator Intake Speed";
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
    private final NetworkTableEntry _funnelIsDropped;
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
    private final SendableChooser<String>  _autoFirstCoralChooser;
    private final SendableChooser<String>  _autoSecondCoralChooser;
    private final SendableChooser<String>  _autoThirdCoralChooser;
    private final SendableChooser<String>  _pathChooser;
    // private String _currentStartPosition;
    // private String _currentFirstCoral;
    // private String _currentSecondCoral;
    // private String _currentThirdCoral;

    // Field
    private final Field2d _field;

    // Auto Paths
    // Need a better solution
    // private static final Pose2d MIDDLE_START_POSITION = new Pose2d(7.553, 4.015,
    // Rotation2d.fromDegrees(180));
    // private static final Pose2d LEFT_START_POSITION = new Pose2d(7.683, 6.676,
    // Rotation2d.fromDegrees(180));
    // private static final Pose2d RIGHT_START_POSITION = new Pose2d(7.159, 2.091,
    // Rotation2d.fromDegrees(180));
    // private static final Map<String, List<String>> NEXT_POSITIONS = new
    // HashMap<String, List<String>>() {
    // {
    // put("A", Arrays.asList("LeftCS", "RightCS"));
    // put("B", Arrays.asList("LeftCS", "RightCS"));
    // put("C", Arrays.asList("RightCS"));
    // put("D", Arrays.asList("RightCS"));
    // put("E", Arrays.asList("RightCS"));
    // put("F", Arrays.asList("RightCS"));
    // put("G", Arrays.asList("RightCS"));
    // put("H", Arrays.asList("LeftCS"));
    // put("I", Arrays.asList("LeftCS"));
    // put("J", Arrays.asList("LeftCS"));
    // put("K", Arrays.asList("LeftCS"));
    // put("L", Arrays.asList("LeftCS"));
    // put("Left", Arrays.asList("I", "J"));
    // put("Middle", Arrays.asList("G", "H"));
    // put("Right", Arrays.asList("E", "F"));
    // put("LeftCS", Arrays.asList("A", "B", "J", "K", "L"));
    // put("RightCS", Arrays.asList("A", "B", "C", "D"));
    // }
    // };

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
        _funnelIsDropped               = NetworkTableInstance.getDefault().getEntry("Dashboard/Robot Values/Funnel Is Dropped");

        _elevatorHeight.setDouble(Constants.Elevator.MIN_EXTENSION);
        _elevatorSetpoint.setDouble(0);
        _manipulatorLeftOutputPercent.setDouble(0);
        _manipulatorRightOutputPercent.setDouble(0);
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
        _autoFirstCoralChooser    = new SendableChooser<String>();
        _autoSecondCoralChooser   = new SendableChooser<String>();
        _autoThirdCoralChooser    = new SendableChooser<String>();
        _pathChooser              = new SendableChooser<String>();

        _autoDelayChooser.setDefaultOption("0", 0);
        _autoDelayChooser.addOption("1", 1);
        _autoDelayChooser.addOption("2", 2);
        _autoDelayChooser.addOption("3", 3);
        _autoDelayChooser.addOption("4", 4);
        _autoDelayChooser.addOption("5", 5);

        _autoStartPositionChooser.addOption("Right", "Right");
        _autoStartPositionChooser.addOption("Middle", "Middle");
        _autoStartPositionChooser.addOption("Left", "Left");

        _autoFirstCoralChooser.addOption("A", "A");
        _autoFirstCoralChooser.addOption("B", "B");
        _autoFirstCoralChooser.addOption("C", "C");
        _autoFirstCoralChooser.addOption("D", "D");
        _autoFirstCoralChooser.addOption("E", "E");
        _autoFirstCoralChooser.addOption("F", "F");
        _autoFirstCoralChooser.addOption("G", "G");
        _autoFirstCoralChooser.addOption("H", "H");
        _autoFirstCoralChooser.addOption("I", "I");
        _autoFirstCoralChooser.addOption("J", "J");
        _autoFirstCoralChooser.addOption("K", "K");
        _autoFirstCoralChooser.addOption("L", "L");

        _autoSecondCoralChooser.addOption("A", "A");
        _autoSecondCoralChooser.addOption("B", "B");
        _autoSecondCoralChooser.addOption("C", "C");
        _autoSecondCoralChooser.addOption("D", "D");
        _autoSecondCoralChooser.addOption("E", "E");
        _autoSecondCoralChooser.addOption("F", "F");
        _autoSecondCoralChooser.addOption("G", "G");
        _autoSecondCoralChooser.addOption("H", "H");
        _autoSecondCoralChooser.addOption("I", "I");
        _autoSecondCoralChooser.addOption("J", "J");
        _autoSecondCoralChooser.addOption("K", "K");
        _autoSecondCoralChooser.addOption("L", "L");

        _autoThirdCoralChooser.addOption("A", "A");
        _autoThirdCoralChooser.addOption("B", "B");
        _autoThirdCoralChooser.addOption("C", "C");
        _autoThirdCoralChooser.addOption("D", "D");
        _autoThirdCoralChooser.addOption("E", "E");
        _autoThirdCoralChooser.addOption("F", "F");
        _autoThirdCoralChooser.addOption("G", "G");
        _autoThirdCoralChooser.addOption("H", "H");
        _autoThirdCoralChooser.addOption("I", "I");
        _autoThirdCoralChooser.addOption("J", "J");
        _autoThirdCoralChooser.addOption("K", "K");
        _autoThirdCoralChooser.addOption("L", "L");

        _pathChooser.setDefaultOption("None", null);
        _pathChooser.addOption("AToLeftCS", "AToLeftCS");
        _pathChooser.addOption("AToRightCS", "AToRightCS");
        _pathChooser.addOption("BToLeftCS", "BToLeftCS");
        _pathChooser.addOption("BToRightCS", "BToRightCS");
        _pathChooser.addOption("CToRightCS", "CToRightCS");
        _pathChooser.addOption("DToRightCS", "DToRightCS");
        _pathChooser.addOption("EToRightCS", "EToRightCS");
        _pathChooser.addOption("FToRightCS", "FToRightCS");
        _pathChooser.addOption("GToRightCS", "GToRightCS");
        _pathChooser.addOption("HToLeftCS", "HToLeftCS");
        _pathChooser.addOption("IToLeftCS", "IToLeftCS");
        _pathChooser.addOption("JToLeftCS", "JToLeftCS");
        _pathChooser.addOption("KToLeftCS", "KToLeftCS");
        _pathChooser.addOption("LeftCSToA", "LeftCSToA");
        _pathChooser.addOption("LeftCSToB", "LeftCSToB");
        _pathChooser.addOption("LeftCSToJ", "LeftCSToJ");
        _pathChooser.addOption("LeftCSToK", "LeftCSToK");
        _pathChooser.addOption("LeftCSToL", "LeftCSToL");
        _pathChooser.addOption("LeftToI", "LeftToI");
        _pathChooser.addOption("LeftToJ", "LeftToJ");
        _pathChooser.addOption("LToLeftCS", "LToLeftCS");
        _pathChooser.addOption("MiddleToG", "MiddleToG");
        _pathChooser.addOption("MiddleToH", "MiddleToH");
        _pathChooser.addOption("RightCSToA", "RightCSToA");
        _pathChooser.addOption("RightCSToB", "RightCSToB");
        _pathChooser.addOption("RightCSToC", "RightCSToC");
        _pathChooser.addOption("RightCSToD", "RightCSToD");
        _pathChooser.addOption("RightToE", "RightToE");
        _pathChooser.addOption("RightToF", "RightToF");

        SmartDashboard.putData("Auto Delay", _autoDelayChooser);
        SmartDashboard.putData("Start Position", _autoStartPositionChooser);
        SmartDashboard.putData("First Coral", _autoFirstCoralChooser);
        SmartDashboard.putData("Second Coral", _autoSecondCoralChooser);
        SmartDashboard.putData("Third Coral", _autoThirdCoralChooser);
        SmartDashboard.putData("Path Chooser", _pathChooser);

        // Field
        _field = new Field2d();

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
        inputs.autoFirstCoral    = _autoFirstCoralChooser.getSelected();
        inputs.autoSecondCoral   = _autoSecondCoralChooser.getSelected();
        inputs.autoThirdCoral    = _autoThirdCoralChooser.getSelected();
        inputs.autoPath          = _pathChooser.getSelected();

        // if (inputs.autoStartPosition != null && inputs.autoStartPosition !=
        // _currentStartPosition)
        // {
        // switch (inputs.autoStartPosition)
        // {
        // case "Middle":
        // _field.setRobotPose(MIDDLE_START_POSITION);
        // setMiddleStartReefOptions(_autoFirstCoralChooser);
        // break;

        // case "Left":
        // _field.setRobotPose(LEFT_START_POSITION);
        // setLeftStartReefOptions(_autoFirstCoralChooser);
        // break;

        // case "Right":
        // _field.setRobotPose(RIGHT_START_POSITION);
        // setRightStartReefOptions(_autoFirstCoralChooser);
        // break;
        // }

        // _currentStartPosition = inputs.autoStartPosition;
        // }
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
        _elevatorZeroStowHeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorL1HeightZeroButton()
    {
        _elevatorZeroL1HeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorL2HeightZeroButton()
    {
        _elevatorZeroL2HeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorL3HeightZeroButton()
    {
        _elevatorZeroL3HeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorL4HeightZeroButton()
    {
        _elevatorZeroL4HeightButton.setBoolean(false);
    }

    @Override
    public void releaseElevatorHangHeightZeroButton()
    {
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

    // private static void setLeftStartReefOptions(SendableSelector<String>
    // selector)
    // {
    // setReefOptions(selector, NEXT_POSITIONS.get("Left"));
    // }

    // private static void setRightStartReefOptions(SendableSelector<String>
    // selector)
    // {
    // setReefOptions(selector, NEXT_POSITIONS.get("Right"));
    // }

    // private static void setMiddleStartReefOptions(SendableSelector<String>
    // selector)
    // {
    // setReefOptions(selector, NEXT_POSITIONS.get("Middle"));
    // }

    // private static void setReefOptions(SendableSelector<String> selector,
    // List<String> options)
    // {
    // selector.clear();

    // for (var option : options)
    // {
    // selector.addOption(option, option);
    // }
    // }

    // // This function assumes we're already at a node
    // // We need to first see which coral station(s) we can go to,
    // // then we can see what nodes we can go to from those coral stations
    // private static List<String> calculateNextAvailablePositions(String current)
    // {
    // var nextPositions = new ArrayList<String>();

    // var coralStations = NEXT_POSITIONS.get(current);

    // for (var coralStation : coralStations)
    // {
    // nextPositions.addAll(NEXT_POSITIONS.get(coralStation));
    // }

    // return nextPositions;
    // }
}
