// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Button;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.BlinkinCommand;
import frc.robot.Commands.ShootCargo;
import frc.robot.Commands.ToggleFlywheel;
import frc.robot.Commands.ToggleIntake;
import frc.robot.Commands.ToggleLowerIndex;
import frc.robot.Commands.ToggleUpperIndex;
import frc.robot.Commands.ToggleWristIn;
import frc.robot.Commands.ToggleWristOut;
import frc.robot.Commands.Auto.Routines.ChampsLeft;
import frc.robot.Commands.Auto.Routines.ChampsRight;
import frc.robot.Commands.Auto.Routines.ChampsScoreandTaxi;
import frc.robot.Commands.Auto.Routines.ChampsTaxiFender;
import frc.robot.Commands.Auto.Routines.ChampsTwoBall;
import frc.robot.Commands.ToggleClimber;
import frc.robot.Commands.TeleOp.PIDNoGyroDrive;
import frc.robot.Commands.TeleOp.TestClimber;
import frc.robot.Commands.TeleOp.TestWrist;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LowerIndex;
import frc.robot.subsystems.UpperIndex;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final LowerIndex lowerIndex = new LowerIndex();
  private final Flywheel flywheel = new Flywheel();
  private final UpperIndex upperIndex = new UpperIndex();
  private final Wrist wrist = new Wrist();
  private final Climber climber = new Climber();
  private final Blinkin blinkin = new Blinkin();
  //private final Limelight limelight = new Limelight();
  private final static SendableChooser<Command> m_autochooser = new SendableChooser<>();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // limelight.setCamMode(CamMode.DRIVER);
    // limelight.setLEDMode(LEDMode.OFF);
    setupDashboard();
    updateAuto();
    // Configure the button bindings
    configureButtonBindings();
    // Configure default commands
    blinkin.setDefaultCommand(
      new BlinkinCommand(blinkin));
    drivetrain.setDefaultCommand(
       new PIDNoGyroDrive(drivetrain, ()-> m_driverController.getLeftY(), ()-> -m_driverController.getRightX()));
    // //operator
    wrist.setDefaultCommand(
      //new ToggleWristIn(wrist));
       new TestWrist(wrist, ()->m_operatorController.getRightY()*.1));
    climber.setDefaultCommand(
        new TestClimber(climber, ()->m_operatorController.getLeftY()*.95));
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    BooleanSupplier driver_rightTrigger = ()-> m_driverController.getRightTriggerAxis() >= .3;
    BooleanSupplier driver_leftTrigger = ()-> m_driverController.getLeftTriggerAxis() >= .3;
    BooleanSupplier d_triggerSafety = ()-> driver_leftTrigger == driver_rightTrigger == true;
    BooleanSupplier operator_rightTrigger = ()-> m_operatorController.getRightTriggerAxis() >= .3;
    BooleanSupplier operator_leftTrigger = ()-> m_operatorController.getLeftTriggerAxis() >= .3;
    boolean o_triggerSafety = operator_leftTrigger == operator_rightTrigger == true;
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whenPressed(() -> drivetrain.halfSpeed = true)
        .whenReleased(() -> drivetrain.halfSpeed = false);
//if(SmartDashboard.getString("Flywheel State", "No Value") == flywheel.getState().string){
    new JoystickButton(m_driverController, Button.kRightBumper.value)
      .whileActiveContinuous(new ShootCargo(upperIndex, lowerIndex, flywheel, m_driverController));
    if(!d_triggerSafety.getAsBoolean()){  //if intake is trying to run foward and reverse, ignore inputs
      new Trigger(driver_rightTrigger)
        .whileActiveContinuous(new ToggleIntake(intake, Intake.State.FWD, wrist, blinkin), true)
        .whileActiveContinuous(new ToggleLowerIndex(lowerIndex, LowerIndex.State.FWD), true)
        .whileActiveContinuous(new ToggleUpperIndex(upperIndex, UpperIndex.State.REV), true)
        //.whileActiveContinuous(new ToggleFlywheel(flywheel, Flywheel.State.REV, m_driverController, m_driverController))
        .whenActive(new ToggleWristOut(wrist))
        //.whileActiveContinuous(()->wrist.setPosition(Wrist.Position.OUT))
        .whenInactive(new ToggleWristIn(wrist));
      new Trigger(driver_leftTrigger)
        .whileActiveContinuous(new ToggleIntake(intake, Intake.State.REV, wrist, blinkin), true)
        .whileActiveContinuous(new ToggleLowerIndex(lowerIndex, LowerIndex.State.REV), true)
        .whileActiveContinuous(new ToggleUpperIndex(upperIndex, UpperIndex.State.REV), true)
        //.whileActiveContinuous(new ToggleFlywheel(flywheel, Flywheel.State.REV, m_driverController, m_driverController))
        .whenActive(new ToggleWristOut(wrist))
        .whenInactive(new ToggleWristIn(wrist));

        //.and(d_rightTrigger).whenInactive(new ToggleWristIn(wrist));
        //.whenInactive(new ToggleWristIn(wrist));
    }
       //operator controls
    new JoystickButton(m_operatorController, Button.kRightBumper.value)
       .toggleWhenPressed(new ToggleFlywheel(flywheel, Flywheel.State.FWD, m_operatorController, m_driverController));
    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
    //.whenActive(()->flywheel.setVoltage(.6*12))
    //.whenInactive(()->flywheel.setVoltage(0));
       .toggleWhenPressed(new ToggleFlywheel(flywheel, Flywheel.State.FULL, m_operatorController, m_driverController));
      //if intake is trying to run foward and reverse, ignore inputs. I'm pretty sure that WPI does this for us anyway
      new Trigger(operator_rightTrigger)
      .whileActiveContinuous(new ToggleIntake(intake, Intake.State.SOFT, wrist, blinkin), true)
      .whileActiveContinuous(new ToggleLowerIndex(lowerIndex, LowerIndex.State.REV), true)
      .whileActiveContinuous(new ToggleUpperIndex(upperIndex, UpperIndex.State.REV), true)
      .whileActiveContinuous(new ToggleFlywheel(flywheel, Flywheel.State.REV, m_driverController, m_driverController))
      .whenActive(new ToggleWristOut(wrist))
      .whenInactive(new ToggleWristIn(wrist));

    new JoystickButton(m_operatorController, Button.kX.value)
      .whenPressed(new ToggleWristIn(wrist));
      
    new JoystickButton(m_operatorController, Button.kB.value)
      .whenPressed(new ToggleClimber(climber, Climber.Position.LOW));
    new JoystickButton(m_operatorController, Button.kY.value)
      .whenPressed(new ToggleClimber(climber, Climber.Position.MID));

    new JoystickButton(m_operatorController, Button.kStart.value)
      .whenPressed(()->climber.m_solenoid.set(Value.kForward));
    new JoystickButton(m_operatorController, Button.kBack.value)
      .whenPressed(()->climber.m_solenoid.set(Value.kReverse));

    new JoystickButton(m_driverController, Button.kBack.value)
       .whenPressed(new ToggleWristIn(wrist));
    new JoystickButton(m_driverController, Button.kStart.value)
       .whenPressed(new ToggleWristOut(wrist));

    new JoystickButton(m_operatorController, Button.kLeftBumper.value)
      .whenReleased(()-> wrist.resetEncoders());
  }
  public void setupDashboard() {
    SmartDashboard.putNumber("Shoot Stall", 1);
    SmartDashboard.putNumber("Drive Stall", 0);
    SmartDashboard.putNumber("Spit Stall", 0);
  }
  public void updateAuto(){
    m_autochooser.setDefaultOption("Champs Left", new ChampsLeft(
      ()->SmartDashboard.getNumber("Shoot Stall", 1), 
      ()->SmartDashboard.getNumber("Drive Stall", 0),
      ()->SmartDashboard.getNumber("Spit Stall", 0),
      blinkin, drivetrain, intake, lowerIndex, upperIndex, flywheel, wrist));
    //m_autochooser.setDefaultOption("Right Three", new Auto1(drivetrain, flywheel, upperIndex, lowerIndex, wrist, intake));
    //m_autochooser.addOption("Left Two", new AutoLeft(drivetrain, flywheel, upperIndex, lowerIndex, wrist, intake));
    m_autochooser.addOption("Champs Right", new ChampsRight(
      ()->SmartDashboard.getNumber("Shoot Stall", 1),
      ()->SmartDashboard.getNumber("Drive Stall", 0),
      ()->SmartDashboard.getNumber("Spit Stall", 0),
      blinkin, drivetrain, intake, lowerIndex, upperIndex, flywheel, wrist));
    //m_autochooser.addOption("Old Left Side", new twoBallSloppy(drivetrain, intake, lowerIndex, upperIndex, flywheel, wrist));
    m_autochooser.addOption("Champs Two Ball", new ChampsTwoBall(
      ()->SmartDashboard.getNumber("Shoot Stall", 1),
      ()->SmartDashboard.getNumber("Drive Stall", 0),
      ()->SmartDashboard.getNumber("Spit Stall", 0),
      blinkin, drivetrain, intake, lowerIndex, upperIndex, flywheel, wrist));
    m_autochooser.addOption("Score+Taxi", new ChampsScoreandTaxi(
      ()->SmartDashboard.getNumber("Shoot Stall", 1),
      ()->SmartDashboard.getNumber("Drive Stall", 0),
      drivetrain, intake, lowerIndex, upperIndex, flywheel, wrist));
    m_autochooser.addOption("Score+Taxi: Fender Edition", new ChampsTaxiFender(
      ()->SmartDashboard.getNumber("Shoot Stall", 1),
      ()->SmartDashboard.getNumber("Drive Stall", 0),
      drivetrain, intake, lowerIndex, upperIndex, flywheel, wrist));

    //m_autochooser.addOption("Score+Taxi", new ScorethenTaxi(drivetrain, intake, lowerIndex, upperIndex, flywheel, wrist));
    SmartDashboard.putData("Auto choices", m_autochooser);
  }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new H1(drivetrain, wrist, flywheel, intake, lowerIndex, upperIndex);
    return m_autochooser.getSelected();
    //return PathLibrary.findPath(drivetrain, "output/test.wpilib.json");
    //return new Auto1(drivetrain, flywheel, upperIndex, lowerIndex, wrist, intake);
    //return AutoLibrary.AutoTest1(drivetrain);
  }
}