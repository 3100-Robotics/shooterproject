// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class RobotContainer {
  private final Shooter shooter = new Shooter();
  private final Indexer indexer = new Indexer();

  private final CommandXboxController driverctl = new CommandXboxController(0);


  public RobotContainer() {
    if (Robot.isSimulation()) {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
    configureBindings();
  }


  private void configureBindings() {
    shooter.setDefaultCommand(shooter.stop());
    indexer.setDefaultCommand(indexer.idle());

    driverctl.a().onTrue(shooter.stop().alongWith(indexer.idle()));
    driverctl.b().onTrue(indexer.wb_intake());
    driverctl.x().onTrue(
      Commands.race(
        shooter.ready(),
        Commands.waitSeconds(1.5)
          .andThen(indexer.wb_release())
          .andThen(Commands.waitSeconds(3))
      ).andThen(shooter.stop())
    );
  }


  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
