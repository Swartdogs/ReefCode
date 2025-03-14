package frc.robot;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
    private final CommandXboxController    _controller              = new CommandXboxController(3); // This is just for testing
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
        Vision.getInstance(Camera.Back);
        Dashboard.getInstance();

        _characterizationChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization());
        _characterizationChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization());
        _characterizationChooser.addOption("Drive SysId (Quasistatic Forward)", Drive.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        _characterizationChooser.addOption("Drive SysId (Quasistatic Reverse)", Drive.getInstance().sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        _characterizationChooser.addOption("Drive SysId (Dynamic Forward)", Drive.getInstance().sysIdDynamic(SysIdRoutine.Direction.kForward));
        _characterizationChooser.addOption("Drive SysId (Dynamic Reverse)", Drive.getInstance().sysIdDynamic(SysIdRoutine.Direction.kReverse));
        _characterizationChooser.addOption("Elevator SysId (Quasistatic Forward)", Elevator.getInstance().sysIdQuasistaticForward());
        _characterizationChooser.addOption("Elevator SysId (Quasistatic Reverse)", Elevator.getInstance().sysIdQuasistaticReverse());
        _characterizationChooser.addOption("Elevator SysId (Dynamic Forward)", Elevator.getInstance().sysIdDynamicForward());
        _characterizationChooser.addOption("Elevator SysId (Dynamic Reverse)", Elevator.getInstance().sysIdDynamicReverse());
        SmartDashboard.putData("Characterization", _characterizationChooser);

        // Configure the button bindings
        configureButtonBindings();
        // configureTestBindings();
    }

    @SuppressWarnings("unused")
    private void configureTestBindings()
    {
        Drive.getInstance().setDefaultCommand(DriveCommands.joystickDrive(() -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> -_controller.getRightX(), () -> false));
        _controller.a().onTrue(DriveCommands.setOdometer(new Pose2d(Units.inchesToMeters(297.5), Units.inchesToMeters(158.5), Rotation2d.fromDegrees(0))));

        _controller.back().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Stow));
        _controller.povUp().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level1));
        _controller.povRight().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level2));
        _controller.povDown().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level3));
        _controller.povLeft().onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level4));

        _controller.leftStick().onTrue(ManipulatorCommands.coralIntake());
        _controller.start().onTrue(CompositeCommands.output());
        _controller.rightStick().onTrue(ManipulatorCommands.stop());

        _controller.rightTrigger().whileTrue(Commands.defer(() -> _characterizationChooser.getSelected(), Set.of(Drive.getInstance(), Elevator.getInstance())));
    }

    @SuppressWarnings("unused")
    private void configureButtonBindings()
    {
        Trigger _driverButton13 = _driverButtons.axisLessThan(0, -0.5);
        Trigger _driverButton14 = _driverButtons.axisGreaterThan(0, -0.5);
        Trigger _driverButton15 = _driverButtons.axisLessThan(1, -0.5);

        // Trigger _hasCoral = new Trigger(() -> _manipulator.hasCoral());
        // Trigger _manipulatorRunning = new Trigger(() -> _manipulator.isRunning());
        Trigger _operatorButton13 = _operatorButtons.axisGreaterThan(0, 0.5);
        Trigger _operatorButton14 = _operatorButtons.axisLessThan(1, -0.5);
        Trigger _operatorButton15 = _operatorButtons.axisGreaterThan(1, 0.5);

        // Default command, normal field-relative drive
        Drive.getInstance().setDefaultCommand(CompositeCommands.joystickDrive(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> -_driverJoystick.getZ(), () -> robotCentric(), 2, 5));

        // Driver Controls
        _driverJoystick.button(2).whileTrue(DriveCommands.reduceSpeed());
        _driverJoystick.button(11).onTrue(DriveCommands.resetGyro());

        _driverJoystick.button(3).whileTrue(ManipulatorCommands.algaeIntake());
        _driverJoystick.button(4).whileTrue(ManipulatorCommands.algaeOutput());

        _driverButtons.button(1).whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_REEF_ANGLE_ONE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(2).whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_REEF_ANGLE_TWO, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(2).whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_REEF_ANGLE_TWO, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(3)
                .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_REEF_ANGLE_THREE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(4).whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_REEF_ANGLE_FOUR, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(5).whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_REEF_ANGLE_FIVE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(6).whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_REEF_ANGLE_SIX, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(7)
                .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_RIGHT_STATION_ANGLE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(8)
                .whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_LEFT_STATION_ANGLE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        _driverButtons.button(9).whileTrue(DriveCommands.driveAtOrientation(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric, Constants.Field.BLUE_PROCESSOR_ANGLE, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // Auto-Align Buttons

        // _driverButtons.button(1).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(2).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'b', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(3).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(4).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(5).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(6).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(7).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(8).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(9).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(10).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(11).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButtons.button(12).whileTrue(CompositeCommands.snapToBranch(Vision.Camera.Front,
        // 'a', () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(),
        // this::robotCentric, Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButton13.whileTrue(DriveCommands.driveAtOrientation(() ->
        // -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric,
        // Constants.Field.BLUE_RIGHT_STATION_ANGLE,
        // Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButton14.whileTrue(DriveCommands.driveAtOrientation(() ->
        // -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric,
        // Constants.Field.BLUE_LEFT_STATION_ANGLE,
        // Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // _driverButton15.whileTrue(DriveCommands.driveAtOrientation(() ->
        // -_driverJoystick.getY(), () -> -_driverJoystick.getX(), this::robotCentric,
        // Constants.Field.BLUE_PROCESSOR_ANGLE,
        // Constants.Drive.MAX_SNAP_SPEED_PERCENTAGE));

        // Operator Controls
        _operatorButtons.button(1).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level4));
        // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // (_hasCoral.getAsBoolean() ? Constants.LED.PURPLE : Constants.LED.RED)),
        // Set.of())));
        _operatorButtons.button(2).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level3));
        // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // (_hasCoral.getAsBoolean() ? Constants.LED.PINK : Constants.LED.RED)),
        // Set.of())));
        _operatorButtons.button(3).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level2));
        // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // (_hasCoral.getAsBoolean() ? Constants.LED.BLUE : Constants.LED.RED)),
        // Set.of())));
        _operatorButtons.button(4).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Level1));
        // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // (_hasCoral.getAsBoolean() ? Constants.LED.ORANGE : Constants.LED.RED)),
        // Set.of())));
        _operatorButtons.button(5).onTrue(ElevatorCommands.setHeight(ElevatorHeight.Stow));
        // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // (_hasCoral.getAsBoolean() ? Constants.LED.GREEN : Constants.LED.RED)),
        // Set.of())));

        _operatorButtons.button(6).onTrue(CompositeCommands.intake());
        _operatorButtons.button(7).onTrue(ManipulatorCommands.stop());
        _operatorButtons.button(8).onTrue(CompositeCommands.output());
        _operatorButtons.button(9).onTrue(ElevatorCommands.modifyHeight(Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        _operatorButtons.button(10).onTrue(ElevatorCommands.modifyHeight(-Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        _operatorButtons.button(11).whileTrue(ManipulatorCommands.algaeIntake());// replace with algae intake
        _operatorButtons.button(12).whileTrue(ManipulatorCommands.algaeOutput());// replace with algaei output
        _operatorButton13.onTrue(ManipulatorCommands.stop()); // replace with algae stop
        (_operatorButton14.or(_operatorButtons.povUp())).whileTrue(ElevatorCommands.hangExecute());
        // .alongWith(LEDCommands.flashColor(Constants.LED.RED));
        // .andThen(LEDCommands.setDefaultColor(Constants.LED.YELLOW))
        // );
        (_operatorButton15.or(_operatorButtons.povDown())).and(_driverJoystick.button(4)).onTrue(FunnelCommands.drop().alongWith(ElevatorCommands.setHeight(ElevatorHeight.Hang)));

        // _hasCoral.onTrue(LEDCommands.setDefaultColor(Constants.LED.GREEN));
        // _hasCoral.onFalse(LEDCommands.setDefaultColor(Constants.LED.RED));
        // _manipulatorRunning.whileTrue(LEDCommands.flashColor(Constants.LED.YELLOW));
    }

    public Command getAutonomousCommand()
    {
        return Dashboard.getInstance().getSelectedAuto();
    }

    private boolean robotCentric()
    {
        return false;
        // return _driverJoystick.button(1).getAsBoolean();
    }
}
