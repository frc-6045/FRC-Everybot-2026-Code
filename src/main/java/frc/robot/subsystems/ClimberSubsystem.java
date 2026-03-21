package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.ClimbConstants.*;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkFlex climberMotor;

  /** Creates a new CANBallSubsystem. */
  public ClimberSubsystem() {
    // create brushed motors for each of the motors on the launcher mechanism
    climberMotor = new SparkMa(CLIMBER_MOTOR_ID, MotorType.kBrushless);

    // create the configuration for the climb moter, set a current limit and apply
    // the config to the controller
    SparkFlexConfig climbConfig = new SparkFlexConfig();
    climbConfig.smartCurrentLimit(CLIMBER_MOTOR_CURRENT_LIMIT);
    climbConfig.idleMode(IdleMode.kBrake);
    climberMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // A method to set the percentage of the climber
  public void setClimber(double power) {
    climberMotor.set(power);
  }

  // A method to stop the climber
  public void stop() {
    climberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
