package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoLaunchSequence;
import frc.robot.commands.ClimbDownTimed;
import frc.robot.commands.ClimbUpTimed;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.EjectTimed;
import frc.robot.commands.IntakeTimed;

public class Autos {
    private SendableChooser<Command> autoChooser;

    public Autos(CANFuelSubsystem fuelSubsystem, ClimberSubsystem climberSubsystem) {
        // Named Commands //
        NamedCommands.registerCommand("AutoLaunchSequence", new AutoLaunchSequence(fuelSubsystem));
        NamedCommands.registerCommand("ClimbUpTimed", new ClimbUpTimed(climberSubsystem, Constants.AutoConstants.CLIMB_UP_SECONDS));
        NamedCommands.registerCommand("ClimbDownTimed", new ClimbDownTimed(climberSubsystem, Constants.AutoConstants.CLIMB_DOWN_SECONDS));
        NamedCommands.registerCommand("EjectTimed", new EjectTimed(fuelSubsystem, Constants.AutoConstants.EJECT_SECONDS));
        NamedCommands.registerCommand("IntakeTimed", new IntakeTimed(fuelSubsystem, Constants.AutoConstants.INTAKE_SECONDS));

        // Autos //
        autoChooser = new SendableChooser<Command>();
        autoChooser.addOption("Shoot twice and climb", AutoBuilder.buildAuto("shootTwiceAndClimb"));
        autoChooser.addOption("Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimb"));
        autoChooser.addOption("Kamikaze (Disperse balls in center)", AutoBuilder.buildAuto("kamikaze (disperse balls in center)"));

        SmartDashboard.putData("autos", autoChooser);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}