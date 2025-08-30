package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import javax.sound.sampled.Line;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;

public class Indexer extends SubsystemBase {
    private final DCMotor GEARBOX = DCMotor.getNEO(1);
    private SparkMax indexer_motor = new SparkMax(0x14, MotorType.kBrushless);

    private SparkBaseConfig indexer_config = new SparkMaxConfig().idleMode(IdleMode.kBrake);

    private final double NOMINAL_SPEED = 1;
    private double speed = NOMINAL_SPEED;

    private String state = "none";

    private DigitalInput beam_break = new DigitalInput(0);
    private DIOSim beam_break_sim = new DIOSim(beam_break);
    private Trigger ballishere = new Trigger(()->beam_break.get());
    private Trigger ballisgone = new Trigger(()->!beam_break.get());

    // Simulation
    private SparkMaxSim indexer_motor_sim = new SparkMaxSim(indexer_motor, GEARBOX);
    private DCMotorSim index_sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            GEARBOX, 
            Constants.Indexer.MOI, 
            2),
        GEARBOX);

    public Indexer() {
        if (Robot.isSimulation()) {
            SmartDashboard.putBoolean("beam_break_sim", false);
        }
        indexer_motor.configure(indexer_config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        SmartDashboard.putNumber("indexer_speed_multiplier", NOMINAL_SPEED);
    }

    @Override
    public void simulationPeriodic() {
        beam_break_sim.setValue(SmartDashboard.getBoolean("beam_break_sim", false));


        // indexer_motor_sim.iterate(NOMINAL_SPEED, speed, NOMINAL_SPEED);
        index_sim.setInput(indexer_motor_sim.getAppliedOutput() * RoboRioSim.getVInVoltage());
        index_sim.update(0.02);
        indexer_motor_sim.iterate(
            Units.radiansPerSecondToRotationsPerMinute(
                index_sim.getAngularVelocityRadPerSec()), 
            RoboRioSim.getVInVoltage(), 0.02
        );

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(index_sim.getCurrentDrawAmps()));

        // Fixme! TODO: AHHHHHH
        SmartDashboard.putNumber("indexer_speed", index_sim.getAngularVelocity().in(RotationsPerSecond));
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("beam_break", beam_break.get());
        SmartDashboard.putString("indexer_command", state);

        speed = MathUtil.clamp(SmartDashboard.getNumber("indexer_speed_multiplier", NOMINAL_SPEED), 0f, 1f);
    }

    public Command wb_intake() {
        return run(()->{indexer_motor.set(speed);state="intaking";})
                .until(ballishere)
                .andThen(runOnce(()->{indexer_motor.stopMotor();state="intaked";}));
    }

    public Command wb_release() {
        return run(()->{indexer_motor.set(speed);state="relasing";})
                .until(ballisgone)
                .andThen(runOnce(()->{indexer_motor.stopMotor();state="reld";}));
    }

    public Command idle() {
        return run(()->{indexer_motor.stopMotor();state="idling";});
    }
}


