package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.Field.Branch;

public final class Utilities
{
    private Utilities()
    {
    }

    public static boolean sparkStickyFault = false;

    public static void ifOk(SparkBase spark, DoubleSupplier supplier, DoubleConsumer consumer)
    {
        double value = supplier.getAsDouble();

        if (spark.getLastError() == REVLibError.kOk)
        {
            consumer.accept(value);
        }
        else
        {
            sparkStickyFault = true;
        }
    }

    public static void ifOk(SparkBase spark, DoubleSupplier[] suppliers, Consumer<double[]> consumer)
    {
        double[] values = new double[suppliers.length];

        for (int i = 0; i < suppliers.length; i++)
        {
            values[i] = suppliers[i].getAsDouble();

            if (spark.getLastError() != REVLibError.kOk)
            {
                sparkStickyFault = true;
                return;
            }
        }

        consumer.accept(values);
    }

    public static void tryUntilOk(SparkBase spark, int maxAttempts, Supplier<REVLibError> command)
    {
        for (int i = 0; i < maxAttempts; i++)
        {
            var error = command.get();

            if (error == REVLibError.kOk)
            {
                break;
            }
            else
            {
                sparkStickyFault = true;
            }
        }
    }

    public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command)
    {
        for (int i = 0; i < maxAttempts; i++)
        {
            var error = command.get();

            if (error.isOK())
            {
                break;
            }
        }
    }

    public static boolean isBlueAlliance()
    {
        var allianceOpt = DriverStation.getAlliance();

        return allianceOpt.isPresent() && allianceOpt.get() == Alliance.Blue;
    }

    public static Pose2d getTagPose(int id)
    {
        var tagPose = Constants.Field.APRIL_TAG_FIELD_LAYOUT.getTagPose(id);

        if (tagPose.isPresent())
        {
            return tagPose.get().toPose2d();
        }
        else
        {
            return new Pose2d();
        }
    }

    public static Branch parseAutoString(String auto)
    {
        String lastChar = auto.substring(auto.length() - 1);

        return switch (lastChar)
        {
            case "A" -> Branch.A;
            case "B" -> Branch.B;
            case "C" -> Branch.C;
            case "D" -> Branch.D;
            case "E" -> Branch.E;
            case "F" -> Branch.F;
            case "G" -> Branch.G;
            case "H" -> Branch.H;
            case "I" -> Branch.I;
            case "J" -> Branch.J;
            case "K" -> Branch.K;
            case "L" -> Branch.L;
            default -> null;
        };
    }
}
