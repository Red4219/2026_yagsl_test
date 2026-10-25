// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.COMMAND_TRAIN_CONSTANTS;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.COMMAND_TRAIN_CONSTANTS.SHOOTER_SPEED;
import frc.robot.commands.AimCommand;
import frc.robot.commands.ToggleAimCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Degrees;
import java.io.File;

import org.dyn4j.dynamics.joint.WheelJoint;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  //private final CommandPS5Controller driverController = new CommandPS5Controller(0);
  private final CommandXboxController driverController = new CommandXboxController(0);
  //private final  CommandPS5Controller driverController2 = new CommandPS5Controller(0);
  private final CommandPS5Controller m_operatorController = new CommandPS5Controller(1);
 
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));

  // /**
  //  * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
  //  */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverController.getLeftY() * 1,
                                                                () -> -driverController.getLeftX() * 1)
                                                                .withControllerRotationAxis(()->driverController.getRightX())
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);


  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> -driverController.getLeftY(),
                                                                () -> -driverController.getLeftX())
                                                                .withControllerRotationAxis(() -> driverController.getRawAxis(
                                                                2))
                                                                .deadband(OperatorConstants.DEADBAND)
                                                                .scaleTranslation(0.8)
                                                                .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                .withControllerHeadingAxis(() ->
                                                                Math.sin(
                                                                driverController.getRawAxis(
                                                                2) *
                                                                Math.PI) *
                                                                (Math.PI *
                                                                2),
                                                                () ->
                                                                Math.cos(
                                                                driverController.getRawAxis(
                                                                2) *
                                                                Math.PI) *
                                                                (Math.PI *
                                                                2))
                                                               .headingWhile(true);
  SendableChooser<Command> autChooser;


  public RobotContainer()
  {
    CameraServer.startAutomaticCapture();
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    autChooser = AutoBuilder.buildAutoChooser("MiddleAuto");
    SmartDashboard.putData("Auto Chooser",autChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandPS5Controller Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */

  private void configureBindings() {
    //driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
    driverController.button(5).whileTrue(new AimCommand(drivebase));
    driverController.button(6).onTrue(new ToggleAimCommand(drivebase));

    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    // /**
    //  * Here we declare all of our operator commands, these commands could have been
    //  * written more compact but are left verbose so the intent is clear.
    //  */

    if (RobotBase.isSimulation()) {
      //drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

      // drivebase.setDefaultCommand(

			// 		new RunCommand(() -> drivebase.drive(
      //       ChassisSpeeds.fromFieldRelativeSpeeds(
      //         new ChassisSpeeds(-driverController.getLeftY(), -driverController.getLeftX(), -driverController.getRawAxis(2)),
      //         drivebase.getHeading()
			// 			)
			// 		), drivebase
      //   )
      // );

    } else {                                                                                                                                 
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autChooser.getSelected();//drivebase.getAutonomousCommand("RightAuto");
  }

  public void setMotorBrake(boolean brake)
  {
   drivebase.setMotorBrake(brake);
  }
}
