package frc.robot.subsystems.dashboard;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public interface DashboardIO
{
    @AutoLog
    public static class DashboardIOInputs
    {
        public double     elevatorMaxHeight        = Constants.Elevator.MAX_EXTENSION;
        public double     elevatorL1Height         = Constants.Elevator.L1_HEIGHT;
        public double     elevatorL2Height         = Constants.Elevator.L2_HEIGHT;
        public double     elevatorL3Height         = Constants.Elevator.L3_HEIGHT;
        public double     elevatorL4Height         = Constants.Elevator.L4_HEIGHT;
        public double     elevatorMinHeight        = 0;
        public double     elevatorStowHeight       = Constants.Elevator.STOW_HEIGHT;
        public Rotation2d flOffset                 = Constants.Drive.FL_ZERO_ROTATION;
        public Rotation2d frOffset                 = Constants.Drive.FR_ZERO_ROTATION;
        public Rotation2d blOffset                 = Constants.Drive.BL_ZERO_ROTATION;
        public Rotation2d brOffset                 = Constants.Drive.BR_ZERO_ROTATION;
        public double     funnelVoltageTime        = Constants.Funnel.DROP_TIME_SECS;
        public double     manipulatorIntakeVoltage = Constants.Manipulator.INTAKE_VOLTS;
        public double     manipulatorOutputVoltage = Constants.Manipulator.OUTPUT_VOLTS;
        public double     elevatorKP               = Constants.Elevator.EXTENSION_KP;
        public double     elevatorKI               = Constants.Elevator.EXTENSION_KI;
        public double     elevatorKD               = Constants.Elevator.EXTENSION_KD;
        public double     elevatorMaxVoltage       = Constants.General.MOTOR_VOLTAGE;
    }

    public default void updateInputs(DashboardIOInputs inputs)
    {

    }

    public default void setRobotPosition(Pose2d pose)
    {

    }

    public default void setMatchTime(double time)
    {

    }

    public default void setHasGamePiece(boolean hasGamePiece)
    {

    }
}
