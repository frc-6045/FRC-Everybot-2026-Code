// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CANFuelSubsystem;

import java.util.function.DoubleSupplier;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HubLaunchSequence extends SequentialCommandGroup {
  /** Creates a new LaunchSequence driven by the controller's right trigger axis. */
  public HubLaunchSequence(CANFuelSubsystem fuelSubsystem, CommandXboxController ctrlr) {
    this(fuelSubsystem, ctrlr::getRightTriggerAxis);
  }

  public HubLaunchSequence(CANFuelSubsystem fuelSubsystem, DoubleSupplier speedSupplier) {
    addCommands(
        //new SpinUp(fuelSubsystem).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(fuelSubsystem, speedSupplier));
  }
}
