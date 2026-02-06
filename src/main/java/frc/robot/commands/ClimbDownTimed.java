package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.Constants.ClimbConstants;

import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.ClimbConstants.CLIMBER_MOTOR_UP_PERCENT;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbDownTimed extends Command {
  /** Creates a new Climber. */

  ClimberSubsystem climberSubsystem;
  private Timer timer;
  final private double timeout;

  public ClimbDownTimed(ClimberSubsystem climberSubsystem, double timeout) {
    addRequirements(climberSubsystem);
    this.climberSubsystem = climberSubsystem;
    this.timeout = timeout;
  }

  // Called when the command is initially scheduled. Set the climber to the
  // appropriate values for unclimbing
  @Override
  public void initialize() {
    climberSubsystem
        .setClimber(ClimbConstants.CLIMBER_MOTOR_DOWN_PERCENT);
    this.timer = new Timer();
    this.timer.reset();
    this.timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted. Stop the climber
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(this.timeout);
  }
}
