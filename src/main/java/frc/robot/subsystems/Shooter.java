package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private SparkMax shooter_motor = new SparkMax(99, MotorType.kBrushless);
    private RelativeEncoder shooter_motor_encoder = shooter_motor.getEncoder();

    private SparkBaseConfig shooter_config = new SparkMaxConfig().idleMode(IdleMode.kCoast);

    private final double NOMINAL_SPEED = 1;
    private double speed = NOMINAL_SPEED;

    // Simulation
    private final DCMotor GEARBOX = DCMotor.getNEO(1);
    private SparkMaxSim shooter_motor_sim = new SparkMaxSim(shooter_motor, GEARBOX);
    private FlywheelSim drum_simulation = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            GEARBOX, Constants.Shooter.MOI, 1), 
            GEARBOX
        );

    public Shooter() {
        SmartDashboard.putNumber("shooter_speed_multiplier", NOMINAL_SPEED);
        shooter_motor.configure(shooter_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    @Override
    public void periodic() {
        speed = MathUtil.clamp(SmartDashboard.getNumber("shooter_speed_multiplier", NOMINAL_SPEED), 0f, 1f);
    }

    @Override
    public void simulationPeriodic() {
        drum_simulation.setInput(shooter_motor_sim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        drum_simulation.update(0.02);
        shooter_motor_sim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(
                drum_simulation.getAngularVelocityRadPerSec()), 
            RoboRioSim.getVInVoltage(), 0.02
        );

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(drum_simulation.getCurrentDrawAmps()));

        // Fixme! TODO: AHHHHHH
        SmartDashboard.putNumber("shooter_speed", drum_simulation.getAngularVelocity().in(RotationsPerSecond));
    }

    public Command ready() {
        return run(()->shooter_motor.set(1*speed));
    }

    public Command stop() {
        return run(()->shooter_motor.stopMotor());
    }
}
