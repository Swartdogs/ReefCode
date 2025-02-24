package frc.robot.subsystems.dashboard;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.CompositeCommands;
import frc.robot.commands.ManipulatorCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator.ElevatorHeight;
import frc.robot.subsystems.manipulator.Manipulator;

public class Dashboard extends SubsystemBase
{
    private static Dashboard _instance;

    public static Dashboard getInstance()
    {
        if (_instance == null)
        {
            var io = switch (Constants.AdvantageKit.CURRENT_MODE)
            {
                case REAL, SIM -> new DashboardIONetwork();
                default -> new DashboardIO() {};
            };

            _instance = new Dashboard(io);
        }

        return _instance;
    }

    public enum DashboardSetting
    {
        FunnelRetractSpeed, FunnelRetractTime, ManipulatorOutputSpeed, ManipulatorIntakeSpeed, ManipulatorL1SpeedMultiplier
    }

    private final DashboardIO                   _io;
    private final DashboardIOInputsAutoLogged   _inputs   = new DashboardIOInputsAutoLogged();
    private final Alert                         _nullAuto;
    private final Map<DashboardSetting, Double> _settings = new LinkedHashMap<DashboardSetting, Double>() {
                                                              {
                                                                  put(DashboardSetting.FunnelRetractSpeed, Constants.Funnel.RETRACT_SPEED);
                                                                  put(DashboardSetting.FunnelRetractTime, Constants.Funnel.DROP_TIME_SECS);
                                                                  put(DashboardSetting.ManipulatorIntakeSpeed, Constants.Manipulator.INTAKE_SPEED);
                                                                  put(DashboardSetting.ManipulatorOutputSpeed, Constants.Manipulator.OUTPUT_SPEED);
                                                                  put(DashboardSetting.ManipulatorL1SpeedMultiplier, Constants.Manipulator.L1_SPEED_MULTIPLIER);
                                                              }
                                                          };
    private Command                             _selectedAuto;
    private Map<String, Command>                _paths;

    private Dashboard(DashboardIO io)
    {
        _io = io;

        _nullAuto = new Alert("No Auto Detected", AlertType.kWarning);

        NamedCommands.registerCommand("ExtendToL1", CompositeCommands.setHeight(ElevatorHeight.Level1));
        NamedCommands.registerCommand("ExtendToL2", CompositeCommands.setHeight(ElevatorHeight.Level2));
        NamedCommands.registerCommand("ExtendToL3", CompositeCommands.setHeight(ElevatorHeight.Level3));
        NamedCommands.registerCommand("ExtendToL4", CompositeCommands.setHeight(ElevatorHeight.Level4));
        NamedCommands.registerCommand("Stow", CompositeCommands.setHeight(ElevatorHeight.Stow));
        NamedCommands.registerCommand("Intake", ManipulatorCommands.intake());
        NamedCommands.registerCommand("Output", CompositeCommands.output());
        NamedCommands.registerCommand("Delay", Commands.defer(() -> Commands.waitSeconds(_inputs.autoDelay), Set.of()));

        try
        {
            Drive.getInstance();

            var AToLeftCS  = PathPlannerPath.fromPathFile("AToLeftCS");
            var AToRightCS = PathPlannerPath.fromPathFile("AToRightCS");
            var BToLeftCS  = PathPlannerPath.fromPathFile("BToLeftCS");
            var BToRightCS = PathPlannerPath.fromPathFile("BToRightCS");
            var CToRightCS = PathPlannerPath.fromPathFile("CToRightCS");
            var DToRightCS = PathPlannerPath.fromPathFile("DToRightCS");
            var EToRightCS = PathPlannerPath.fromPathFile("EToRightCS");
            var FToRightCS = PathPlannerPath.fromPathFile("FToRightCS");
            var GToRightCS = PathPlannerPath.fromPathFile("GToRightCS");
            var HToLeftCS  = PathPlannerPath.fromPathFile("HToLeftCS");
            var IToLeftCS  = PathPlannerPath.fromPathFile("IToLeftCS");
            var JToLeftCS  = PathPlannerPath.fromPathFile("JToLeftCS");
            var KToLeftCS  = PathPlannerPath.fromPathFile("KToLeftCS");
            var LeftCSToA  = PathPlannerPath.fromPathFile("LeftCSToA");
            var LeftCSToB  = PathPlannerPath.fromPathFile("LeftCSToB");
            var LeftCSToJ  = PathPlannerPath.fromPathFile("LeftCSToJ");
            var LeftCSToK  = PathPlannerPath.fromPathFile("LeftCSToK");
            var LeftCSToL  = PathPlannerPath.fromPathFile("LeftCSToL");
            var LeftToI    = PathPlannerPath.fromPathFile("LeftToI");
            var LeftToJ    = PathPlannerPath.fromPathFile("LeftToJ");
            var LToLeftCS  = PathPlannerPath.fromPathFile("LToLeftCS");
            var MiddleToG  = PathPlannerPath.fromPathFile("MiddleToG");
            var MiddleToH  = PathPlannerPath.fromPathFile("MiddleToH");
            var RightCSToA = PathPlannerPath.fromPathFile("RightCSToA");
            var RightCSToB = PathPlannerPath.fromPathFile("RightCSToB");
            var RightCSToC = PathPlannerPath.fromPathFile("RightCSToC");
            var RightCSToD = PathPlannerPath.fromPathFile("RightCSToD");
            var RightToE   = PathPlannerPath.fromPathFile("RightToE");
            var RightToF   = PathPlannerPath.fromPathFile("RightToF");

            _paths = new LinkedHashMap<>() {
                {

                    put("AToLeftCS", Commands.sequence(AutoBuilder.resetOdom(AToLeftCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(AToLeftCS)));
                    put("AToRightCS", Commands.sequence(AutoBuilder.resetOdom(AToRightCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(AToRightCS)));
                    put("BToLeftCS", Commands.sequence(AutoBuilder.resetOdom(BToLeftCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(BToLeftCS)));
                    put("BToRightCS", Commands.sequence(AutoBuilder.resetOdom(BToRightCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(BToRightCS)));
                    put("CToRightCS", Commands.sequence(AutoBuilder.resetOdom(CToRightCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(CToRightCS)));
                    put("DToRightCS", Commands.sequence(AutoBuilder.resetOdom(DToRightCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(DToRightCS)));
                    put("EToRightCS", Commands.sequence(AutoBuilder.resetOdom(EToRightCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(EToRightCS)));
                    put("FToRightCS", Commands.sequence(AutoBuilder.resetOdom(FToRightCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(FToRightCS)));
                    put("GToRightCS", Commands.sequence(AutoBuilder.resetOdom(GToRightCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(GToRightCS)));
                    put("HToLeftCS", Commands.sequence(AutoBuilder.resetOdom(HToLeftCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(HToLeftCS)));
                    put("IToLeftCS", Commands.sequence(AutoBuilder.resetOdom(IToLeftCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(IToLeftCS)));
                    put("JToLeftCS", Commands.sequence(AutoBuilder.resetOdom(JToLeftCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(JToLeftCS)));
                    put("KToLeftCS", Commands.sequence(AutoBuilder.resetOdom(KToLeftCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(KToLeftCS)));
                    put("LeftCSToA", Commands.sequence(AutoBuilder.resetOdom(LeftCSToA.getStartingHolonomicPose().get()), AutoBuilder.followPath(LeftCSToA)));
                    put("LeftCSToB", Commands.sequence(AutoBuilder.resetOdom(LeftCSToB.getStartingHolonomicPose().get()), AutoBuilder.followPath(LeftCSToB)));
                    put("LeftCSToJ", Commands.sequence(AutoBuilder.resetOdom(LeftCSToJ.getStartingHolonomicPose().get()), AutoBuilder.followPath(LeftCSToJ)));
                    put("LeftCSToK", Commands.sequence(AutoBuilder.resetOdom(LeftCSToK.getStartingHolonomicPose().get()), AutoBuilder.followPath(LeftCSToK)));
                    put("LeftCSToL", Commands.sequence(AutoBuilder.resetOdom(LeftCSToL.getStartingHolonomicPose().get()), AutoBuilder.followPath(LeftCSToL)));
                    put("LeftToI", Commands.sequence(AutoBuilder.resetOdom(LeftToI.getStartingHolonomicPose().get()), AutoBuilder.followPath(LeftToI)));
                    put("LeftToJ", Commands.sequence(AutoBuilder.resetOdom(LeftToJ.getStartingHolonomicPose().get()), AutoBuilder.followPath(LeftToJ)));
                    put("LToLeftCS", Commands.sequence(AutoBuilder.resetOdom(LToLeftCS.getStartingHolonomicPose().get()), AutoBuilder.followPath(LToLeftCS)));
                    put("MiddleToG", Commands.sequence(AutoBuilder.resetOdom(MiddleToG.getStartingHolonomicPose().get()), AutoBuilder.followPath(MiddleToG)));
                    put("MiddleToH", Commands.sequence(AutoBuilder.resetOdom(MiddleToH.getStartingHolonomicPose().get()), AutoBuilder.followPath(MiddleToH)));
                    put("RightCSToA", Commands.sequence(AutoBuilder.resetOdom(RightCSToA.getStartingHolonomicPose().get()), AutoBuilder.followPath(RightCSToA)));
                    put("RightCSToB", Commands.sequence(AutoBuilder.resetOdom(RightCSToB.getStartingHolonomicPose().get()), AutoBuilder.followPath(RightCSToB)));
                    put("RightCSToC", Commands.sequence(AutoBuilder.resetOdom(RightCSToC.getStartingHolonomicPose().get()), AutoBuilder.followPath(RightCSToC)));
                    put("RightCSToD", Commands.sequence(AutoBuilder.resetOdom(RightCSToD.getStartingHolonomicPose().get()), AutoBuilder.followPath(RightCSToD)));
                    put("RightToE", Commands.sequence(AutoBuilder.resetOdom(RightToE.getStartingHolonomicPose().get()), AutoBuilder.followPath(RightToE)));
                    put("RightToF", Commands.sequence(AutoBuilder.resetOdom(RightToF.getStartingHolonomicPose().get()), AutoBuilder.followPath(RightToF)));
                }
            };
        }
        catch (Exception e)
        {
            e.printStackTrace();
            _paths = null;
        }

    }

    @Override
    public void periodic()
    {
        _io.updateInputs(_inputs);
        Logger.processInputs("Dashboard", _inputs);

        // Robot Values
        _io.setManipulatorLeftMotorOutputPercentSpeed(Manipulator.getInstance().getLeftOutputSpeed());
        _io.setManipulatorRightMotorOutputPercentSpeed(Manipulator.getInstance().getRightOutputSpeed());

        // Update settings
        _settings.put(DashboardSetting.FunnelRetractSpeed, _inputs.funnelRetractPercentSpeed);
        _settings.put(DashboardSetting.FunnelRetractTime, _inputs.funnelRetractTime);
        _settings.put(DashboardSetting.ManipulatorIntakeSpeed, _inputs.manipulatorIntakePercentSpeed);
        _settings.put(DashboardSetting.ManipulatorOutputSpeed, _inputs.manipulatorOutputPercentSpeed);
        _settings.put(DashboardSetting.ManipulatorL1SpeedMultiplier, _inputs.manipulatorL1SpeedMultiplier);

        // Buttons
        if (_inputs.elevatorZeroMinHeightPressed)
        {
            _io.releaseElevatorMinHeightZeroButton();
        }

        if (_inputs.elevatorZeroMaxHeightPressed)
        {
            _io.releaseElevatorMaxHeightZeroButton();
        }

        if (_inputs.elevatorZeroStowHeightPressed)
        {
            _io.releaseElevatorStowHeightZeroButton();
        }

        if (_inputs.elevatorZeroL1HeightPressed)
        {
            _io.releaseElevatorL1HeightZeroButton();
        }

        if (_inputs.elevatorZeroL2HeightPressed)
        {
            _io.releaseElevatorL2HeightZeroButton();
        }

        if (_inputs.elevatorZeroL3HeightPressed)
        {
            _io.releaseElevatorL3HeightZeroButton();
        }

        if (_inputs.elevatorZeroL4HeightPressed)
        {
            _io.releaseElevatorL4HeightZeroButton();
        }

        if (_inputs.elevatorZeroHangHeightPressed)
        {
            _io.releaseElevatorHangHeightZeroButton();
        }

        if (_inputs.driveZeroFLModulePressed)
        {
            _io.releaseDriveFLOffsetZeroButton();
        }

        if (_inputs.driveZeroFRModulePressed)
        {
            _io.releaseDriveFROffsetZeroButton();
        }

        if (_inputs.driveZeroBLModulePressed)
        {
            _io.releaseDriveBLOffsetZeroButton();
        }

        if (_inputs.driveZeroBRModulePressed)
        {
            _io.releaseDriveBROffsetZeroButton();
        }

        if (_inputs.driveZeroModulesPressed)
        {
            _io.releaseDriveModuleOffsetZeroButton();
        }

        if (_inputs.autoPath != null)
        {
            _selectedAuto = _paths.get(_inputs.autoPath);
        }
        else
        {
            _selectedAuto = Constants.Lookups.AUTO_LOOKUP.get(_inputs.autoStartPosition + _inputs.autoFirstCoral + _inputs.autoSecondCoral + _inputs.autoThirdCoral);
        }
        _nullAuto.set(_selectedAuto == null);
    }

    public double getSetting(DashboardSetting setting)
    {
        return _settings.get(setting);
    }

    public Command getSelectedAuto()
    {
        return _selectedAuto;
    }
}
