package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.funnel.Funnel;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer
{
    // Controller
    private final CommandJoystick       _driverJoystick  = new CommandJoystick(0);
    private final CommandJoystick       _driverButtons   = new CommandJoystick(1);
    private final CommandJoystick       _operatorButtons = new CommandJoystick(2);
    private final CommandXboxController _controller      = new CommandXboxController(3); // This is just for testing

    // Dashboard inputs
    private final LoggedDashboardChooser<String>  _startingPositionChooser;
    private final LoggedDashboardChooser<String>  _firstCoralChooser;
    private final LoggedDashboardChooser<String>  _secondCoralChooser;
    private final LoggedDashboardChooser<String>  _thirdCoralChooser;
    private final LoggedDashboardChooser<Integer> _autoDelayChooser;

    //
    private final Alert _nullAuto;
    private Command     _selectedAuto;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer()
    {
        NamedCommands.registerCommand("ExtendToL1", ElevatorCommands.setHeight(ElevatorHeight.Level1));
        NamedCommands.registerCommand("ExtendToL2", ElevatorCommands.setHeight(ElevatorHeight.Level2));
        NamedCommands.registerCommand("ExtendToL3", ElevatorCommands.setHeight(ElevatorHeight.Level3));
        NamedCommands.registerCommand("ExtendToL4", ElevatorCommands.setHeight(ElevatorHeight.Level4));
        NamedCommands.registerCommand("Stow", ElevatorCommands.setHeight(ElevatorHeight.Stow));
        NamedCommands.registerCommand("Intake", ManipulatorCommands.intake());
        NamedCommands.registerCommand("Output", CompositeCommands.output());
        NamedCommands.registerCommand("Delay", Commands.defer(() -> Commands.waitSeconds(autoDelayTime()), Set.of()));

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

        // Configure the button bindings
        configureButtonBindings();
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

        _controller.leftStick().onTrue(ManipulatorCommands.intake());
        _controller.start().onTrue(ManipulatorCommands.output());
        _controller.rightStick().onTrue(ManipulatorCommands.stop());
    }

    @SuppressWarnings("unused")
    private void configureButtonBindings()
    {
        // Trigger _hasCoral = new Trigger(() -> _manipulator.hasCoral());
        // Trigger _manipulatorRunning = new Trigger(() -> _manipulator.isRunning());
        // Trigger _operatorButton12 = _operatorButtons.axisGreaterThan(0, 0.5);
        // Trigger _operatorButton13 = _operatorButtons.axisGreaterThan(1, 0.5);
        // Trigger _operatorButton14 = _operatorButtons.axisLessThan(1, -0.5);

        // Default command, normal field-relative drive
        Drive.getInstance().setDefaultCommand(DriveCommands.joystickDrive(() -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> -_driverJoystick.getZ(), () -> robotCentric()));
        // _drive.setDefaultCommand(CompositeCommands.joystickDrive(_drive, _elevator,
        // () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // -_driverJoystick.getZ(), () -> robotCentric(), 2, 3));

        // Driver Controls
        // _driverJoystick.button(2).onTrue(null); // replace this with switching
        // cameras/
        _driverJoystick.button(2).whileTrue(DriveCommands.reduceSpeed());
        _driverJoystick.button(11).onTrue(DriveCommands.resetGyro());

        // _driverButtons.button(0).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_ONE :
        // Constants.Field.BLUE_REEF_ANGLE_ONE,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );
        // _driverButtons.button(1).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_TWO :
        // Constants.Field.BLUE_REEF_ANGLE_TWO,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );
        // _driverButtons.button(2).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_THREE
        // : Constants.Field.BLUE_REEF_ANGLE_THREE,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );
        // _driverButtons.button(3).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_FOUR :
        // Constants.Field.BLUE_REEF_ANGLE_FOUR,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );
        // _driverButtons.button(4).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_FIVE :
        // Constants.Field.BLUE_REEF_ANGLE_FIVE,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );
        // _driverButtons.button(5).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_SIX :
        // Constants.Field.BLUE_REEF_ANGLE_SIX,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );
        // _driverButtons.button(6).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ?
        // Constants.Field.RED_LEFT_STATION_ANGLE :
        // Constants.Field.BLUE_LEFT_STATION_ANGLE,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );
        // _driverButtons.button(7).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ?
        // Constants.Field.RED_RIGHT_STATION_ANGLE :
        // Constants.Field.BLUE_RIGHT_STATION_ANGLE,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );
        // _driverButtons.button(8).whileTrue(
        // DriveCommands.driveAtOrientation(
        // _drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () ->
        // robotCentric(), () -> isRedAlliance() ? Constants.Field.RED_PROCESSOR_ANGLE :
        // Constants.Field.BLUE_PROCESSOR_ANGLE,
        // Constants.Drive.MAX_SNAP_SPEED
        // )
        // );

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

        _operatorButtons.button(6).onTrue(ManipulatorCommands.intake());
        _operatorButtons.button(7).onTrue(ManipulatorCommands.stop());
        _operatorButtons.button(8).onTrue(CompositeCommands.output());
        _operatorButtons.button(9).onTrue(ElevatorCommands.modifyHeight(Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        _operatorButtons.button(10).onTrue(ElevatorCommands.modifyHeight(-Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        // _operatorButtons.button(11).onTrue(null);// replace with algae intake
        // _operatorButtons.button(12).onTrue(null);// replace with algae output
        // _operatorButton12.onTrue(null);// replace with algae stop
        // _operatorButton13.and(_driverJoystick.button(3)).onTrue(FunnelCommands.drop(_funnel).alongWith(ElevatorCommands.setHeight(_elevator,
        // ElevatorHeight.Hang)));
        // .alongWith(LEDCommands.flashColor(_led, Constants.LED.RED)).until(() ->
        // _elevator.atSetpoint())
        // .andThen(LEDCommands.setDefaultColor(_led, Constants.LED.YELLOW))
        // );
        // _operatorButton14.onTrue(ElevatorCommands.hangExecute(_elevator));

        // _hasCoral.onTrue(LEDCommands.setDefaultColor(_led, Constants.LED.GREEN));
        // _hasCoral.onFalse(LEDCommands.setDefaultColor(_led, Constants.LED.RED));

        // _manipulatorRunning.whileTrue(LEDCommands.flashColor(_led,
        // Constants.LED.YELLOW));
    }

    public boolean getFunnelIsDropped()
    {
        return Funnel.getInstance().isDropped();
    }

    public void periodic()
    {
        // var cedricAuto = _dashboard.getPaths("Left_JL");
        // _dashboard.setAuto(cedricAuto);
        // _dashboard.setRobotPosition(cedricAuto.get(0).getStartingHolonomicPose().get());

        _selectedAuto = Constants.Lookups._lookup.get(_startingPositionChooser.get() + _firstCoralChooser.get() + _secondCoralChooser.get() + _thirdCoralChooser.get());
        _nullAuto.set(_selectedAuto == null);
    }

    public static boolean isRedAlliance()
    {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public Command getAutonomousCommand()
    {
        return _selectedAuto;
    }

    public Integer autoDelayTime()
    {
        return _autoDelayChooser.get();
    }

    public boolean robotCentric()
    {
        return _driverJoystick.button(1).getAsBoolean();
    }
}
