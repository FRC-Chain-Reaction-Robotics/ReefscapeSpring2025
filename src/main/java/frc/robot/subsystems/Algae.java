package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Algae {
    private SparkMax intakeMotor;
    private SparkMax angleMotor;
    private DigitalInput upLimitSwitch, downLimitSwitch;
    /*
     * Uncomment when ready to tune and rename variables
     */
    // private LinearSystem<N2, N1, N2> elevatorPlant =
    // LinearSystemId.identifyPositionSystem(-1, -1);

    // @SuppressWarnings("unchecked")
    // private KalmanFilter<N2, N1, N1> filter = new KalmanFilter<>(
    // Nat.N2(),
    // Nat.N1(),
    // (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
    // VecBuilder.fill(0.015, 0.17), //System estimate accuracy in meters and meters
    // per second
    // VecBuilder.fill(0.001), //Input accuracy (the encoder)
    // 0.020);

    // @SuppressWarnings("unchecked")
    // private LinearQuadraticRegulator<N2, N1, N1> controller = new
    // LinearQuadraticRegulator<>(
    // (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
    // VecBuilder.fill(0, 0), // State error tolerance in meters and meters per
    // second
    // VecBuilder.fill(0), // Control effort in volts
    // 0.020);

    // @SuppressWarnings("unchecked")
    // private LinearSystemLoop<N2, N1, N1> elevatorLoop = new LinearSystemLoop<>(
    // (LinearSystem<N2, N1, N1>) elevatorPlant.slice(0),
    // controller,
    // filter,
    // 12.0,
    // 0.020);

    public Algae() {
        intakeMotor = new SparkMax(21, MotorType.kBrushless);
        angleMotor = new SparkMax(22, MotorType.kBrushless);
        upLimitSwitch = new DigitalInput(Constants.Algae.upLimitSwitch);
        downLimitSwitch = new DigitalInput(Constants.Algae.downLimitSwitch);
    }

    public void off() {
        intakeMotor.set(0.0);
        angleMotor.set(0.0);
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

}
