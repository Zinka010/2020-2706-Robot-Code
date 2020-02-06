package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class SpinUpShooter extends CommandBase {
  
  private ShooterSubsystem shooterSubsystem;

  public SpinUpShooter() {
    shooterSubsystem = ShooterSubsystem.getInstance();
    addRequirements(shooterSubsystem);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {
    // Spin to win
    shooterSubsystem.periodic();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
