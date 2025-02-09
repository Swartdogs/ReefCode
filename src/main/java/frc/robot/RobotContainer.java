package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.FunnelCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.elevator.Elevator;
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

public class RobotContainer
{
    // Subsystems
    private final Drive       _drive;
    private final Elevator    _elevator;
    private final Manipulator _manipulator;
    private final Funnel      _funnel;
    // private final LED _led;

    // Controller
    private final CommandXboxController _controller = new CommandXboxController(0);

    // Dashboard inputs
    // private final LoggedDashboardChooser<Command> _autoChooser;

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
                // _led = new LED(new LEDIOHardware());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _elevator = new Elevator(new ElevatorIOSim());
                _manipulator = new Manipulator(new ManipulatorIOSim(() -> _controller.leftTrigger().getAsBoolean()));
                _funnel = new Funnel(new FunnelIOSim());
                // _led = new LED(new LEDIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _elevator = new Elevator(new ElevatorIO() {});
                _manipulator = new Manipulator(new ManipulatorIO() {});
                _funnel = new Funnel(new FunnelIO() {});
                // _led = new LED(new LEDIO() {});
                break;
        }

        // Set up auto routines
        // _autoChooser = new LoggedDashboardChooser<>("Auto Choices",
        // AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        // _autoChooser.addOption("Drive Wheel Radius Characterization",
        // DriveCommands.wheelRadiusCharacterization(_drive));
        // _autoChooser.addOption("Drive Simple FF Characterization",
        // DriveCommands.feedforwardCharacterization(_drive));
        // _autoChooser.addOption("Drive SysId (Quasistatic Forward)",
        // _drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // _autoChooser.addOption("Drive SysId (Quasistatic Reverse)",
        // _drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        // _autoChooser.addOption("Drive SysId (Dynamic Forward)",
        // _drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // _autoChooser.addOption("Drive SysId (Dynamic Reverse)",
        // _drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> -_controller.getRightX()));
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

        _controller.y().onTrue(FunnelCommands.drop(_funnel));
        // .alongWith(ElevatorCommands.setHeight(_elevator,
        // ElevatorHeight.Hang)).alongWith(LEDCommands.flashColor(_led,
        // Constants.LED.RED)).until(() -> _elevator.atSetpoint())
        // .andThen(LEDCommands.setDefaultColor(_led, Constants.LED.YELLOW))
        // );
        // _controller.rightTrigger().whileTrue(ElevatorCommands.setVolts(_elevator, ()
        // -> -_controller.getRightY() * Constants.General.MOTOR_VOLTAGE));

        // _controller.back()
        // .onTrue(ElevatorCommands.setHeight(_elevator,
        // ElevatorHeight.Stow).alongWith(new DeferredCommand(() ->
        // LEDCommands.setDefaultColor(_led, (_hasCoral.getAsBoolean() ?
        // Constants.LED.GREEN : Constants.LED.RED)), Set.of())));
        // _controller.povUp()
        // .onTrue(ElevatorCommands.setHeight(_elevator,
        // ElevatorHeight.Level1).alongWith(new DeferredCommand(() ->
        // LEDCommands.setDefaultColor(_led, (_hasCoral.getAsBoolean() ?
        // Constants.LED.PURPLE : Constants.LED.RED)), Set.of())));
        // _controller.povRight()
        // .onTrue(ElevatorCommands.setHeight(_elevator,
        // ElevatorHeight.Level2).alongWith(new DeferredCommand(() ->
        // LEDCommands.setDefaultColor(_led, (_hasCoral.getAsBoolean() ?
        // Constants.LED.PINK : Constants.LED.RED)), Set.of())));
        // _controller.povDown()
        // .onTrue(ElevatorCommands.setHeight(_elevator,
        // ElevatorHeight.Level3).alongWith(new DeferredCommand(() ->
        // LEDCommands.setDefaultColor(_led, (_hasCoral.getAsBoolean() ?
        // Constants.LED.BLUE : Constants.LED.RED)), Set.of())));
        // _controller.povLeft()
        // .onTrue(ElevatorCommands.setHeight(_elevator,
        // ElevatorHeight.Level4).alongWith(new DeferredCommand(() ->
        // LEDCommands.setDefaultColor(_led, (_hasCoral.getAsBoolean() ?
        // Constants.LED.ORANGE : Constants.LED.RED)), Set.of())));

        _controller.leftTrigger().onTrue(ManipulatorCommands.intake(_manipulator));
        _controller.start().onTrue(ManipulatorCommands.output(_manipulator));
        _controller.rightStick().onTrue(ManipulatorCommands.stop(_manipulator));

        // _hasCoral.onTrue(LEDCommands.setDefaultColor(_led, Constants.LED.GREEN));
        // _hasCoral.onFalse(LEDCommands.setDefaultColor(_led, Constants.LED.RED));

        // _manipulatorRunning.whileTrue(LEDCommands.flashColor(_led,
        // Constants.LED.YELLOW));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }
}
