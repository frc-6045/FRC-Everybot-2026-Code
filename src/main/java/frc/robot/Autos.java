package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoLaunchSequence;
import frc.robot.subsystems.CANFuelSubsystem;

public class Autos {
    private SendableChooser<Command> autoChooser;

    public Autos(CANFuelSubsystem fuelSubsystem) {
        // Named Commands //
        NamedCommands.registerCommand("AutoLaunchSequence", new AutoLaunchSequence(fuelSubsystem));

        // Autos //
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("Shoot twice and climb", AutoBuilder.buildAuto("shootTwiceAndClimb"));

        SmartDashboard.putData("autos", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}