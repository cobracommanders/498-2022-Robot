package frc.robot.Commands.TeleOp;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PIDDrive extends CommandBase{
  private final Drivetrain drivetrain;
  private final DoubleSupplier speed;
  private final DoubleSupplier rotation;
  double s, r;
  public PIDDrive(Drivetrain drivetrain, DoubleSupplier speed, DoubleSupplier rotation) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.rotation = rotation;
    addRequirements(this.drivetrain);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    s = speed.getAsDouble();
    r = rotation.getAsDouble();
    drivetrain.setPID(s+r, s-r);
    //SmartDashboard.putNumber("Left Drive Velocity", drivetrain.leftWheelSpeedMetersPerSecond());
    //SmartDashboard.putNumber("Right Drive Velocity", drivetrain.rightWheelSpeedMetersPerSecond());

    //SmartDashboard.putNumber("Left Stator Current", drivetrain.m_left_1.getStatorCurrent());
    //SmartDashboard.putNumber("Left Supply Current", drivetrain.m_left_1.getSupplyCurrent());

    //SmartDashboard.putNumber("Drive Heading", drivetrain.getHeading().getDegrees());
    //SmartDashboard.putNumber("Drive Total Velocity", drivetrain.getAbsoluteVelocity());
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }
}
