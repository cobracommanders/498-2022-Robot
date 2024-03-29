package frc.robot.Commands.TeleOp;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class PIDNoGyroDrive extends CommandBase{
  private final Drivetrain drivetrain;
  private final DoubleSupplier speed;
  private final DoubleSupplier rotation;
  double s, r;
  SlewRateLimiter drive_limiter = new SlewRateLimiter(5);
  public PIDNoGyroDrive(Drivetrain drivetrain, DoubleSupplier speed, DoubleSupplier rotation) {
    this.drivetrain = drivetrain;
    this.speed = speed;
    this.rotation = rotation;
    addRequirements(this.drivetrain);
  }
  @Override 
  public void initialize(){
    drivetrain.setNeutralMode(NeutralMode.Coast);
  }
  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
      r = rotation.getAsDouble();//* DriveConstants.maxSpeedMetersPerSec;
      s = drive_limiter.calculate(speed.getAsDouble());// * DriveConstants.maxSpeedMetersPerSec;
    if(Math.abs(speed.getAsDouble())<= .2){
      s = 0;
    }
    if(Math.abs(rotation.getAsDouble())<=.2){
      r = 0;
    }
    s = s*s * Math.signum(s);
    r = r*r*Math.signum(r);
    drivetrain.setPIDNoGyro(s+r, s-r);
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
