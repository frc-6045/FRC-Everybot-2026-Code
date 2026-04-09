// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANFuelSubsystem;
import static frc.robot.Constants.FuelConstants.*;

import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Launch extends Command {
  /** Creates a new Intake. */

  CANFuelSubsystem fuelSubsystem;
  DoubleSupplier speedSupplier;

  public Launch(CANFuelSubsystem fuelSystem, DoubleSupplier speedSupplier) {
    addRequirements(fuelSystem);
    this.fuelSubsystem = fuelSystem;
    this.speedSupplier = speedSupplier;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for intaking
  @Override
  public void initialize() {
    applyRollerOutputs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  // Re-reads the supplier so the launcher tracks the trigger live.
  @Override
  public void execute() {
    applyRollerOutputs();
  }

  private void applyRollerOutputs() {
    double speed = speedSupplier.getAsDouble();
    double launcherOutput = SmartDashboard.getNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT) * speed;
    double feederOutput = SmartDashboard.getNumber("Launching feeder roller value", -INDEXER_LAUNCHING_PERCENT) * speed;
    fuelSubsystem.setIntakeLauncherRoller(launcherOutput);
    fuelSubsystem.setFeederRoller(feederOutput);
    SmartDashboard.putNumber("Launching launcher modified roller value", launcherOutput);
    SmartDashboard.putNumber("Launching feeder modified roller value", feederOutput);
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
