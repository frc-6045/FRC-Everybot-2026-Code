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

@SuppressWarnings("unused")
public class Autos {
    @SuppressWarnings("FieldMayBeFinal")
    private SendableChooser<Command> autoChooser;

    public Autos(CANFuelSubsystem fuelSubsystem, ClimberSubsystem climberSubsystem) {
        // Named Commands //
        NamedCommands.registerCommand("AutoLaunchSequence", new AutoLaunchSequence(fuelSubsystem));
        NamedCommands.registerCommand("ClimbUpTimed", new ClimbUpTimed(climberSubsystem, Constants.AutoConstants.CLIMB_UP_SECONDS));
        NamedCommands.registerCommand("ClimbDownTimed", new ClimbDownTimed(climberSubsystem, Constants.AutoConstants.CLIMB_DOWN_SECONDS));
        NamedCommands.registerCommand("EjectTimed", new EjectTimed(fuelSubsystem, Constants.AutoConstants.EJECT_SECONDS));
        NamedCommands.registerCommand("IntakeTimed", new IntakeTimed(fuelSubsystem, Constants.AutoConstants.INTAKE_SECONDS));

        // Autos //
        //noinspection MoveFieldAssignmentToInitializer,Convert2Diamond
        autoChooser = new SendableChooser<Command>();
        createAuto("shootTwiceAndClimb", "Shoot twice and climb", true, true, autoChooser);
        createAuto("shootOnceAndClimb", "Shoot once and climb", true, true, autoChooser);
        createAuto("kamikaze", "Kamikaze", true, false, autoChooser);

        //autoChooser.addOption("[L] Shoot twice and climb", AutoBuilder.buildAuto("shootTwiceAndClimbLeft"));
        //autoChooser.addOption("[M] Shoot twice and climb", AutoBuilder.buildAuto("shootTwiceAndClimb"));
        //autoChooser.addOption("[R] Shoot twice and climb", AutoBuilder.buildAuto("shootTwiceAndClimbRight"));
        //autoChooser.addOption("[L] [CL] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimbLeftCLeft"));
        //autoChooser.addOption("[L] [CM] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimbLeft"));
        //autoChooser.addOption("[L] [CR] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimbLeftCRight"));
        //autoChooser.addOption("[M] [CL] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimbCLeft"));
        //autoChooser.addOption("[M] [CM] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimb"));
        //autoChooser.addOption("[M] [CR] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimbCRight"));
        //autoChooser.addOption("[R] [CL] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimbRightCLeft"));
        //autoChooser.addOption("[R] [CM] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimbRight"));
        //autoChooser.addOption("[R] [CR] Shoot once and climb", AutoBuilder.buildAuto("shootOnceAndClimbRightCRight"));
        //autoChooser.addOption("[L] Kamikaze", AutoBuilder.buildAuto("kamikazeLeft"));
        //autoChooser.addOption("[M] Kamikaze", AutoBuilder.buildAuto("kamikaze (disperse balls in center)"));
        //autoChooser.addOption("[R] Kamikaze", AutoBuilder.buildAuto("kamikazeRight"));

        SmartDashboard.putData("autos", autoChooser);
    }

    @SuppressWarnings("SameParameterValue")
    private void createAuto(String name, String friendlyName, Boolean hasPositionVariants, Boolean hasClimbVariants, SendableChooser<Command> autoChooser) {
        if (hasPositionVariants) {
            if (hasClimbVariants) {
                autoChooser.addOption("[L] [CL] " + friendlyName, AutoBuilder.buildAuto(name+"LeftCLeft"));
                autoChooser.addOption("[M] [CL] " + friendlyName, AutoBuilder.buildAuto(name+"CLeft"));
                autoChooser.addOption("[R] [CL] " + friendlyName, AutoBuilder.buildAuto(name+"RightCLeft"));
                autoChooser.addOption("[L] [CM] " + friendlyName, AutoBuilder.buildAuto(name+"Left"));
                autoChooser.addOption("[M] [CM] " + friendlyName, AutoBuilder.buildAuto(name));
                autoChooser.addOption("[R] [CM] " + friendlyName, AutoBuilder.buildAuto(name+"Right"));
                autoChooser.addOption("[L] [CR] " + friendlyName, AutoBuilder.buildAuto(name+"LeftCRight"));
                autoChooser.addOption("[M] [CR] " + friendlyName, AutoBuilder.buildAuto(name+"CRight"));
                autoChooser.addOption("[R] [CR] " + friendlyName, AutoBuilder.buildAuto(name+"RightCRight"));
            } else {
                autoChooser.addOption("[L] " + friendlyName, AutoBuilder.buildAuto(name+"Left"));
                autoChooser.addOption("[M] " + friendlyName, AutoBuilder.buildAuto(name));
                autoChooser.addOption("[R] " + friendlyName, AutoBuilder.buildAuto(name+"Right"));
            }
        } else {
            autoChooser.addOption(friendlyName, AutoBuilder.buildAuto(name));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}