package frc.robot.subsystems;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private SparkMax sparkMax1;
    private DigitalInput forwardLimitSwitch, reverseLimitSwitch;

    // private LinearSystem<N2, N1, N2> elevatorPlant = LinearSystemId.identifyPositionSystem(-1, -1);

    // @SuppressWarnings("unchecked")
    // private KalmanFilter<N2, N1, N1> filter = new KalmanFilter<>(
    //     Nat.N2(), 
    //     Nat.N1(), 
    //     (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0), 
    //     VecBuilder.fill(0.015, 0.17), //System estimate accuracy in meters and meters per second
    //     VecBuilder.fill(0.001), //Input accuracy (the encoder)
    //     0.020);

    // @SuppressWarnings("unchecked")
    // private LinearQuadraticRegulator<N2, N1, N1> controller = new LinearQuadraticRegulator<>(
    //     (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0), 
    //     VecBuilder.fill(0, 0), // State error tolerance in meters and meters per second
    //     VecBuilder.fill(0), // Control effort in volts
    //     0.020);

    // @SuppressWarnings("unchecked")
    // private LinearSystemLoop<N2, N1, N1> elevatorLoop = new LinearSystemLoop<>(
    //     (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
    //     controller,
    //     filter,
    //     12.0,
    //     0.020);

    public Elevator() {
        // Orignal IDs:
        // SparkMax1: 15
        // SparkMax2: 16
        sparkMax1 = new SparkMax(18, MotorType.kBrushless);

        SparkBaseConfig config1 = new SparkMaxConfig().inverted(true);
        sparkMax1.configure(config1, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        // forwardLimitSwitch = new DigitalInput(Constants.Elevator.forwardLimitSwitch);
        // reverseLimitSwitch = new DigitalInput(Constants.Elevator.reverseLimitSwitch);
    }

    public void up(double speed) {
        // if (forwardLimitSwitch.get()) {
            sparkMax1.set(speed);
        // } else {
        //     sparkMax1.set(0.0);
        // }
    }

    public void down(double speed) {
        // if (reverseLimitSwitch.get()) {
            sparkMax1.set(-speed);
        // } else {
        //     sparkMax1.set(0.0);
        // }
    }

    public void off() {
        sparkMax1.set(0.0);
    }

    public Command elevatorCommand(IntSupplier in) {
        return null;
    }

    @Deprecated
    public Command elevatorCommand(BooleanSupplier up, BooleanSupplier down) {
        return run(() -> {
            if (!(down.getAsBoolean() || up.getAsBoolean())) {
                off();
            } else if (down.getAsBoolean()) {
                down(0.75);
            } else {
                up(0.755);
            }
        });
    }
}
