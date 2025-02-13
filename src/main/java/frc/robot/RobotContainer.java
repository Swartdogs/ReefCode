package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.FunnelCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.dashboard.Dashboard;
import frc.robot.subsystems.dashboard.DashboardIO;
import frc.robot.subsystems.dashboard.DashboardIONetwork;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOHardware;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelIO;
import frc.robot.subsystems.funnel.FunnelIOHardware;
import frc.robot.subsystems.funnel.FunnelIOSim;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOHardware;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;

import java.util.Set;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer
{
    // Subsystems
    private final Drive       _drive;
    private final Elevator    _elevator;
    private final Manipulator _manipulator;
    private final Funnel      _funnel;
    private final Dashboard   _dashboard;
    // private final LED         _led;

    // Controller
    private final CommandJoystick _driverJoystick  = new CommandJoystick(0);
    private final CommandJoystick _driverButtons   = new CommandJoystick(1);
    private final CommandJoystick _operatorButtons = new CommandJoystick(2);

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
        switch (Constants.AdvantageKit.CURRENT_MODE)
        {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                _drive = new Drive(new GyroIONavX(), new ModuleIOHardware(0), new ModuleIOHardware(1), new ModuleIOHardware(2), new ModuleIOHardware(3));
                _elevator = new Elevator(new ElevatorIOHardware());
                _manipulator = new Manipulator(new ManipulatorIOHardware());
                _funnel = new Funnel(new FunnelIOHardware());
                _dashboard = new Dashboard(new DashboardIONetwork());
                // _led = new LED(new LEDIOHardware());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _elevator = new Elevator(new ElevatorIOSim());
                _manipulator = new Manipulator(new ManipulatorIOSim(() -> _driverJoystick.button(7).getAsBoolean()));
                _funnel = new Funnel(new FunnelIOSim());
                _dashboard = new Dashboard(new DashboardIONetwork());
                // _led = new LED(new LEDIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _elevator = new Elevator(new ElevatorIO() {});
                _manipulator = new Manipulator(new ManipulatorIO() {});
                _funnel = new Funnel(new FunnelIO() {});
                _dashboard = new Dashboard(new DashboardIO() {});
                // _led = new LED(new LEDIO() {});
                break;
        }

        NamedCommands.registerCommand("ExtendToL1", ElevatorCommands.setHeight(_elevator, Constants.Elevator.L1_HEIGHT));
        NamedCommands.registerCommand("ExtendToL2", ElevatorCommands.setHeight(_elevator, Constants.Elevator.L2_HEIGHT));
        NamedCommands.registerCommand("ExtendToL3", ElevatorCommands.setHeight(_elevator, Constants.Elevator.L3_HEIGHT));
        NamedCommands.registerCommand("ExtendToL4", ElevatorCommands.setHeight(_elevator, Constants.Elevator.L4_HEIGHT));
        NamedCommands.registerCommand("Stow", ElevatorCommands.setHeight(_elevator, Constants.Elevator.STOW_HEIGHT));

        if (RobotBase.isSimulation())
        {
            NamedCommands.registerCommand("Intake", Commands.none());
        }
        else
        {
            NamedCommands.registerCommand("Intake", ManipulatorCommands.intake(_manipulator));
        }

        NamedCommands.registerCommand("Output", ManipulatorCommands.output(_manipulator));
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

    private void configureButtonBindings()
    {
        // _funnel.setDefaultCommand(FunnelCommands.setVolts(_funnel, () ->
        // Constants.Funnel.DEFAULT_VOLTS));
        // _elevator.setDefaultCommand(ElevatorCommands.setVolts(_elevator, () ->
        // MathUtil.applyDeadband(-_controller.getRightY(), 0.1) *
        // Constants.General.MOTOR_VOLTAGE));

        // Trigger _hasCoral = new Trigger(() -> _manipulator.hasCoral());
        // Trigger _manipulatorRunning = new Trigger(() -> _manipulator.isRunning());

        // Default command, normal field-relative drive
        _drive.setDefaultCommand(DriveCommands.joystickFieldCentricDrive(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> -_controller.getRightX()));
        // _drive.setDefaultCommand(DriveCommands.setTurnOpenLoop(_drive, () ->
        // MathUtil.applyDeadband(-_controller.getRightX(), 0.1) * 4.0));
        // _controller.a().onTrue(DriveCommands.setTurnPosition(_drive,
        // Rotation2d.fromDegrees(0)));
        // _controller.b().onTrue(DriveCommands.setTurnPosition(_drive,
        // Rotation2d.fromDegrees(90)));
        // _controller.y().onTrue(DriveCommands.setTurnPosition(_drive,
        // Rotation2d.fromDegrees(180)));
        // _controller.x().onTrue(DriveCommands.setTurnPosition(_drive,
        // Rotation2d.fromDegrees(270)));

        // Lock to 0° when A button is held
        // _controller.a().whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () ->
        // -_controller.getLeftY(), () -> -_controller.getLeftX(), () ->
        // new-_controller.getRightX()
        // Rotation2d()));

        // Reset gyro to 0° when B button is pressed
        // _controller.b().onTrue(Commands.runOnce(() -> _drive.setPose(new
        // Pose2d(_drive.getPose().getTranslation(), new Rotation2d())),
        // _drive).ignoringDisable(true));

        // _controller.y().onTrue(FunnelCommands.drop(_funnel));
        // .alongWith(ElevatorCommands.setHeight(_elevator,
        // ElevatorHeight.Hang)).alongWith(LEDCommands.flashColor(_led,
        // Constants.LED.RED)).until(() -> _elevator.atSetpoint())
        // .andThen(LEDCommands.setDefaultColor(_led, Constants.LED.YELLOW))
        // );
        // _controller.rightTrigger().whileTrue(ElevatorCommands.setVolts(_elevator, ()
        // -> -_controller.getRightY() * Constants.General.MOTOR_VOLTAGE));

        _controller.back().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Stow));
        // // .alongWith(new DeferredCommand(() -> LEDCommandsetDefaultColor(_led,
        // // (_hasCoral.getAsBoolean() ? Constants.LED.GREEN : Constants.LED.RED)),
        // // Set.of())));
        _controller.povUp().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level1));
        // // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // // (_hasCoral.getAsBoolean() ?
        // // Constants.LED.PURPLE : Constants.LED.RED)), Set.of())));
        _controller.povRight().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level2));
        // // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // // (_hasCoral.getAsBoolean() ?
        // // Constants.LED.PINK : Constants.LED.RED)), Set.of())));
        _controller.povDown().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level3));
        // // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // // (_hasCoral.getAsBoolean() ?
        // // Constants.LED.BLUE : Constants.LED.RED)), Set.of())));
        _controller.povLeft().onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level4));
        // // .alongWith(new DeferredCommand(() -> LEDCommands.setDefaultColor(_led,
        // // (_hasCoral.getAsBoolean() ?
        // // Constants.LED.ORANGE : Constants.LED.RED)), Set.of())));

        _controller.leftStick().onTrue(ManipulatorCommands.intake(_manipulator));
        _controller.start().onTrue(ManipulatorCommands.output(_manipulator));
        _controller.rightStick().onTrue(ManipulatorCommands.stop(_manipulator));

        // _hasCoral.onTrue(LEDCommands.setDefaultColor(_led, Constants.LED.GREEN));
        // _hasCoral.onFalse(LEDCommands.setDefaultColor(_led, Constants.LED.RED));

        // _manipulatorRunning.whileTrue(LEDCommands.flashColor(_led,
        // Constants.LED.YELLOW));
    }

    public boolean getFunnelIsDropped()
    {
        return _funnel.isDropped();
    }

    public void periodic()
    {
        // var cedricAuto = _dashboard.getPaths("Left_JL");
        // _dashboard.setAuto(cedricAuto);
        // _dashboard.setRobotPosition(cedricAuto.get(0).getStartingHolonomicPose().get());

        _selectedAuto = Constants.Lookups._lookup.get(_startingPositionChooser.get() + _firstCoralChooser.get() + _secondCoralChooser.get() + _thirdCoralChooser.get());
        _nullAuto.set(_selectedAuto == null);
    }

    public boolean isRedAlliance()
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
}
