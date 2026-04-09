// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

/**
 * Runs the launcher at a fixed setpoint (LAUNCH_FROM_HUB_PERCENT) to score
 * consistently from the hub. Spins up the launcher rollers first, then engages
 * the feeder to push fuel through. Runs while held.
 */
public class LaunchFromHub extends Command {
  private final CANFuelSubsystem fuelSubsystem;
  private final Timer timer = new Timer();
  private boolean feedingStarted;

  public LaunchFromHub(CANFuelSubsystem fuelSystem) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
  }

  @Override
  public void initialize() {
    timer.restart();
    feedingStarted = false;
    // Read the dashboard tunable once per press — release and re-press to pick up new values.
    fuelSubsystem.setIntakeLauncherRoller(
        SmartDashboard.getNumber("Launch from hub launcher value", LAUNCH_FROM_HUB_PERCENT));
    fuelSubsystem.setFeederRoller(-INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);
  }

  @Override
  public void execute() {
    if (!feedingStarted && timer.hasElapsed(SPIN_UP_SECONDS)) {
      fuelSubsystem.setFeederRoller(-INDEXER_LAUNCHING_PERCENT);
      feedingStarted = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    fuelSubsystem.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
