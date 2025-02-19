package frc.robot.util;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;

import edu.wpi.first.wpilibj.DriverStation;

public final class Utilities
{
    private Utilities()
    {}

    public static boolean sparkStickyFault = false;

    public static boolean isRedAlliance()
    {
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
    }

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
}
