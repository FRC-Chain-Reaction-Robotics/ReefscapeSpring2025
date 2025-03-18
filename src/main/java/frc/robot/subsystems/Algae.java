package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Algae extends SubsystemBase {
    private SparkFlex intakeMotor, angleMotor;
    private DigitalInput upLimitSwitch, downLimitSwitch;

    private final TrapezoidProfile m_profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                    Units.degreesToRadians(40),
                    Units.degreesToRadians(80)));

    private LinearSystem<N2, N1, N2> algaePlant =
    LinearSystemId.createSingleJointedArmSystem(DCMotor.getNeoVortex(1), Constants.Algae.kArmMOI, 
        Constants.Algae.kArmGearing);

    @SuppressWarnings("unchecked")
    private KalmanFilter<N2, N1, N1> filter = new KalmanFilter<>(
    Nat.N2(),
    Nat.N1(),
    (LinearSystem<N2, N1, N1>) algaePlant.slice(0),
    VecBuilder.fill(0.015, 0.17), //System estimate accuracy in radians and radians
    // per second
    VecBuilder.fill(0.001), //Input accuracy (the encoder)
    0.020);

    @SuppressWarnings("unchecked")
    private LinearQuadraticRegulator<N2, N1, N1> controller = new
    LinearQuadraticRegulator<>(
    (LinearSystem<N2, N1, N1>) algaePlant.slice(0),
    VecBuilder.fill(0.2, 0.05), // State error tolerance in radians and radians per
    // second
    VecBuilder.fill(4), // Control effort in volts
    0.020);

    @SuppressWarnings("unchecked")
    private LinearSystemLoop<N2, N1, N1> elevatorLoop = new LinearSystemLoop<>(
    (LinearSystem<N2, N1, N1>) algaePlant.slice(0),
    controller,
    filter,
    12.0,
    0.020);

    public Algae() {
        intakeMotor = new SparkFlex(19, MotorType.kBrushless);
        angleMotor = new SparkFlex(17, MotorType.kBrushless);
        // upLimitSwitch = new DigitalInput(Constants.Algae.upLimitSwitch);
        // downLimitSwitch = new DigitalInput(Constants.Algae.downLimitSwitch);
    }

    public void rotateoff() {
        angleMotor.set(0.0);
    }

    public void stopIntake() {
        intakeMotor.set(0);
    }

    public void intake(double speed) {
        intakeMotor.set(-speed);
    }

    public void outtake(double speed) {
        intakeMotor.set(speed);
    }

    public void rotateup(double speed) {
        angleMotor.set(speed);
    }

    public void rotatedown(double speed) {
        angleMotor.set(-speed);
    }

    public Command defaultCommand(BooleanSupplier up, BooleanSupplier down, BooleanSupplier in, BooleanSupplier out) {
        return run(() -> {
            // This upper if-else-if-else structure is old and should be replaced with linearSystem loop
            if (!(down.getAsBoolean() || up.getAsBoolean())) {
                rotateoff();
            } else if (down.getAsBoolean()) {
                rotatedown(0.5);
            } else {
                rotateup(0.5);
            }

            if (!(in.getAsBoolean() || out.getAsBoolean())) {
                stopIntake();
            } else if (in.getAsBoolean()) {
                intake(0.5);
            } else {
                outtake(0.5);
            }
        });
    }
}
