package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;
import frc.robot.subsystems.manipulator.ManipulatorIO;
import frc.robot.subsystems.manipulator.ManipulatorIOHardware;
import frc.robot.subsystems.manipulator.ManipulatorIOSim;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer
{
    // Subsystems
    private final Drive       _drive;
    private final Elevator    _elevator;
    private final Manipulator _manipulator;
    private final Funnel      _funnel;

    // Controller
    private final CommandJoystick _driverJoystick  = new CommandJoystick(0);
    private final CommandJoystick _driverButtons   = new CommandJoystick(1);
    private final CommandJoystick _operatorButtons = new CommandJoystick(2);

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
                _elevator = new Elevator(new ElevatorIOHardware());
                _manipulator = new Manipulator(new ManipulatorIOHardware());
                _funnel = new Funnel(new FunnelIOHardware());
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
                _elevator = new Elevator(new ElevatorIOSim());
                _manipulator = new Manipulator(new ManipulatorIOSim(() -> _driverJoystick.button(7).getAsBoolean()));
                _funnel = new Funnel(new FunnelIOSim());
                break;

            default:
                // Replayed robot, disable IO implementations
                _drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                _elevator = new Elevator(new ElevatorIO() {});
                _manipulator = new Manipulator(new ManipulatorIO() {});
                _funnel = new Funnel(new FunnelIO() {});
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
        _drive.setDefaultCommand(DriveCommands.joystickFieldCentricDrive(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> -_driverJoystick.getZ()));

        _driverJoystick.button(1).whileTrue(DriveCommands.joystickRobotCentricDrive(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> -_driverJoystick.getZ()));
        _driverJoystick.button(2).onTrue(null); // replace this with switching cameras
        _driverJoystick.button(11).onTrue(DriveCommands.resetGyro(_drive, () -> isRedAlliance()));

        _driverButtons.button(0)
                .whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_ONE : Constants.Field.BLUE_REEF_ANGLE_ONE));
        _driverButtons.button(1)
                .whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_TWO : Constants.Field.BLUE_REEF_ANGLE_TWO));
        _driverButtons.button(2)
                .whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_THREE : Constants.Field.BLUE_REEF_ANGLE_THREE));
        _driverButtons.button(3)
                .whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_FOUR : Constants.Field.BLUE_REEF_ANGLE_FOUR));
        _driverButtons.button(4)
                .whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_FIVE : Constants.Field.BLUE_REEF_ANGLE_FIVE));
        _driverButtons.button(5)
                .whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_REEF_ANGLE_SIX : Constants.Field.BLUE_REEF_ANGLE_SIX));
        _driverButtons.button(6)
                .whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_LEFT_STATION_ANGLE : Constants.Field.BLUE_LEFT_STATION_ANGLE));
        _driverButtons.button(7).whileTrue(
                DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_RIGHT_STATION_ANGLE : Constants.Field.BLUE_RIGHT_STATION_ANGLE)
        );
        _driverButtons.button(8)
                .whileTrue(DriveCommands.joystickDriveAtAngle(_drive, () -> -_driverJoystick.getY(), () -> -_driverJoystick.getX(), () -> isRedAlliance() ? Constants.Field.RED_PROCESSOR_ANGLE : Constants.Field.BLUE_PROCESSOR_ANGLE));

        _operatorButtons.button(0).onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level1));
        _operatorButtons.button(1).onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level2));
        _operatorButtons.button(2).onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level3));
        _operatorButtons.button(3).onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Level4));
        _operatorButtons.button(4).onTrue(ElevatorCommands.setHeight(_elevator, ElevatorHeight.Stow));
        _operatorButtons.button(5).onTrue(ManipulatorCommands.intake(_manipulator));
        _operatorButtons.button(6).onTrue(ManipulatorCommands.output(_manipulator));
        _operatorButtons.button(7).onTrue(ManipulatorCommands.stop(_manipulator));
        _operatorButtons.button(8).and(_driverJoystick.button(3)).onTrue(FunnelCommands.drop(_funnel)); // replace with hang prep
        _operatorButtons.button(9).onTrue(null);// replace with hang execute
        _operatorButtons.button(10).onTrue(null);// replace with algae intake
        _operatorButtons.button(11).onTrue(null);// replace with algae output

        Trigger _operatorButton12 = _operatorButtons.axisGreaterThan(0, 0.5);
        Trigger _operatorButton13 = _operatorButtons.axisGreaterThan(1, 0.5);
        Trigger _operatorButton14 = _operatorButtons.axisLessThan(1, -0.5);

        _operatorButton12.onTrue(null);// replace with algae stop
        _operatorButton13.onTrue(ElevatorCommands.modifyHeight(_elevator, Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));
        _operatorButton14.onTrue(ElevatorCommands.modifyHeight(_elevator, -Constants.Elevator.ELEVATOR_MODIFICATION_HEIGHT));

    }

    public boolean isRedAlliance()
    {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

    public Command getAutonomousCommand()
    {
        return _autoChooser.get();
    }
}
