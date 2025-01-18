package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOHardware;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOHardware;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer
{
    // Subsystems
    private final Drive       _drive;
    private final Manipulator _manipulator;

    // Controller
    private final CommandXboxController _controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> _autoChooser;

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
                _manipulator = new Manipulator(new ManipulatorIOHardware());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _manipulator = new Manipulator(new ManipulatorIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _manipulator = new Manipulator(new ManipulatorIO() {});
                break;
        }

        // Set up auto routines
        _autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        _autoChooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(_drive));
        _autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(_drive));
        _autoChooser.addOption("Drive SysId (Quasistatic Forward)", _drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        _autoChooser.addOption("Drive SysId (Quasistatic Reverse)", _drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        _autoChooser.addOption("Drive SysId (Dynamic Forward)", _drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        _autoChooser.addOption("Drive SysId (Dynamic Reverse)", _drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Configure the button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings()
    {
        // Default command, normal field-relative drive
        _drive.setDefaultCommand(DriveCommands.joystickDrive(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> -_controller.getRightX()));

        // Lock to 0° when A button is held
        _controller.a().whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_controller.getLeftY(), () -> -_controller.getLeftX(), () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        _controller.x().onTrue(Commands.runOnce(_drive::stopWithX, _drive));

        // Reset gyro to 0° when B button is pressed
        _controller.b().onTrue(Commands.runOnce(() -> _drive.setPose(new Pose2d(_drive.getPose().getTranslation(), new Rotation2d())), _drive).ignoringDisable(true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return _autoChooser.get();
    }
}
