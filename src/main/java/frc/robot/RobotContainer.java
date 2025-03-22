package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Field.Branch;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.FunnelCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.Camera;

public class RobotContainer
{
    // Controller
    private final CommandJoystick          _driverJoystick          = new CommandJoystick(0);
    private final CommandJoystick          _driverButtons           = new CommandJoystick(1);
    private final CommandJoystick          _operatorButtons         = new CommandJoystick(2);
    private final CommandXboxController    _controller              = new CommandXboxController(3);
    private final SendableChooser<Command> _characterizationChooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        DriverStation.silenceJoystickConnectionWarning(true);

        Drive.getInstance();
        Elevator.getInstance();
        Manipulator.getInstance();
        Funnel.getInstance();
        Vision.getInstance(Camera.Front);
        Vision.getInstance(Camera.FrontCenter);
        Dashboard.getInstance();

        // Configure the button bindings
        configureButtonBindings();
        // configureTestBindings();
    }

    @SuppressWarnings("unused")
    private void configureTestBindings()
    {
        Drive.getInstance().setDefaultCommand(DriveCommands.joystickDrive(() -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> -_controller.getRightX(), () -> false));
        // _controller.a().onTrue(DriveCommands.setOdometer(new
        // Pose2d(Units.inchesToMeters(297.5), Units.inchesToMeters(158.5),
        // Rotation2d.fromDegrees(0))));

        _controller.back().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Stow));
        _controller.povUp().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level1));
        _controller.povRight().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level2));
        _controller.povDown().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level3));
        _controller.povLeft().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level4));

        _controller.start().whileTrue(
                CompositeCommands.autoAlign(Branch.C, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );
    }

    @SuppressWarnings("unused")
    private void configureButtonBindings()
    {
        Trigger funnelDropped = new Trigger(() -> Funnel.getInstance().isDropped());

        Trigger driverButton13 = _driverButtons.axisLessThan(0, -0.5);
        Trigger driverButton14 = _driverButtons.axisGreaterThan(0, 0.5);
        Trigger driverButton15 = _driverButtons.axisLessThan(1, -0.5);

        // Trigger _hasCoral = new Trigger(() -> _manipulator.hasCoral());
        // Trigger _manipulatorRunning = new Trigger(() -> _manipulator.isRunning());
        Trigger operatorButton13 = _operatorButtons.axisGreaterThan(0, 0.5);
        Trigger operatorButton14 = _operatorButtons.axisGreaterThan(1, 0.5);
        Trigger operatorButton15 = _operatorButtons.axisLessThan(1, -0.5);

        // Default command, normal field-relative drive
        Drive.getInstance().setDefaultCommand(CompositeCommands.joystickDrive(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> -_driverJoystick.getZ(), () -> robotCentric(), 2, 5));

        // Driver Controls
        (_driverJoystick.button(1).and(funnelDropped)).whileTrue(ElevatorCommands.hangExecute());
        _driverJoystick.button(2).whileTrue(DriveCommands.reduceSpeed());
        _driverJoystick.button(11).onTrue(DriveCommands.resetGyro());
        _driverJoystick.button(12).whileTrue(
                CompositeCommands.autoAlign(Branch.C, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        // Auto-Align Buttons

        _driverButtons.button(1).whileTrue(
                CompositeCommands.autoAlign(Branch.A, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(2).whileTrue(
                CompositeCommands.autoAlign(Branch.B, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(3).whileTrue(
                CompositeCommands.autoAlign(Branch.C, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(4).whileTrue(
                CompositeCommands.autoAlign(Branch.D, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(5).whileTrue(
                CompositeCommands.autoAlign(Branch.E, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(6).whileTrue(
                CompositeCommands.autoAlign(Branch.F, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(7).whileTrue(
                CompositeCommands.autoAlign(Branch.G, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(8).whileTrue(
                CompositeCommands.autoAlign(Branch.H, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(9).whileTrue(
                CompositeCommands.autoAlign(Branch.I, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(10).whileTrue(
                CompositeCommands.autoAlign(Branch.J, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(11).whileTrue(
                CompositeCommands.autoAlign(Branch.K, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        _driverButtons.button(12).whileTrue(
                CompositeCommands.autoAlign(Branch.L, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Drive.MAX_AUTO_TRANSLATE_SPEED_PERCENTAGE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE)
        );

        driverButton13.whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_RIGHT_STATION_ANGLE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));
        driverButton14.whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_LEFT_STATION_ANGLE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));
        driverButton15.whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_PROCESSOR_ANGLE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // Operator Controls
        _operatorButtons.button(1).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level4));
        _operatorButtons.button(2).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level3));
        _operatorButtons.button(3).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level2));
        _operatorButtons.button(4).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level1));
        _operatorButtons.button(5).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Stow));
        _operatorButtons.button(6).onTrue(CompositeCommands.intake());
        _operatorButtons.button(7).onTrue(ManipulatorCommands.stop());
        _operatorButtons.button(8).onTrue(CompositeCommands.output());
        _operatorButtons.button(9).whileTrue(ManipulatorCommands.algaeIntake());
        _operatorButtons.button(10).whileTrue(ManipulatorCommands.algaeOutput());
        _operatorButtons.button(11).onTrue(ElevatorCommands.setHeight(ElevatorHeight.HighAlgae));
        _operatorButtons.button(11).onTrue(ElevatorCommands.setHeight(ElevatorHeight.LowAlgae));
        operatorButton13.onTrue(ElevatorCommands.modifyHeight(Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        operatorButton14.onTrue(ElevatorCommands.modifyHeight(-Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        (operatorButton15.or(_operatorButtons.povDown())).and(_driverJoystick.button(4)).onTrue(FunnelCommands.drop().alongWith(ElevatorCommands.setHeight(ElevatorHeight.Hang)));

        // _hasCoral.onTrue(LEDCommands.setDefaultColor(Constants.LED.GREEN));
        // _hasCoral.onFalse(LEDCommands.setDefaultColor(Constants.LED.RED));
        // _manipulatorRunning.whileTrue(LEDCommands.flashColor(Constants.LED.YELLOW));

        _characterizationChooser.setDefaultOption("Do nothing", Commands.none());
        _characterizationChooser.addOption("Quasistatic Forward", Drive.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        _characterizationChooser.addOption("Quasistatic Reverse", Drive.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        _characterizationChooser.addOption("Dynamic Forward", Drive.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
        _characterizationChooser.addOption("Dynamic Reverse", Drive.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));

        SmartDashboard.putData("Characterization Chooser", _characterizationChooser);
    }

    public Command getAutonomousCommand()
    {
        return Dashboard.getInstance().getSelectedAuto();
    }

    private boolean robotCentric()
    {
        return false;
    }
}
