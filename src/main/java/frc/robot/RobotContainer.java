// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.IOConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.IntakeClass;
import frc.robot.subsystems.Loader;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
@Logged
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
    private final Flywheel m_flywheel = new Flywheel();
    private final IntakeClass m_intake = new IntakeClass();
    private final Loader m_loader = new Loader();
  

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController m_driverController =
        new CommandXboxController(OperatorConstants.kDriverControllerPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the trigger bindings
        configureBindings();
    }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
    private void configureBindings() {

        // Control the drive with split-stick arcade controls
        m_driveSubsystem.setDefaultCommand(
            m_driveSubsystem.arcadeDriveCommand(
                () -> -m_driverController.getLeftY(), () -> -m_driverController.getRightX()));

        /* TO DO: Add bindings here */
        m_driverController.back().and(m_driverController.y())
            .whileTrue(m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        m_driverController.back().and(m_driverController.a())
            .whileTrue(m_flywheel.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        m_driverController.start().and(m_driverController.y())
            .whileTrue(m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kForward));
        m_driverController.start().and(m_driverController.a())
            .whileTrue(m_flywheel.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // 1-meter hub shooting presets for the Slot0/VelocityVoltage closed loop
        // (Flywheel.runAtRPM()), tested between 3000 and 4000 RPM from 1 meter off the hub.
        m_driverController.leftBumper().whileTrue(m_flywheel.runAtRPM(3250));
        m_driverController.rightBumper().whileTrue(m_flywheel.runAtRPM(3500));
        m_driverController.x().whileTrue(m_flywheel.runAtRPM(3750));

        // Ball-feed testing: hold B to run the indexer (Loader) and intake together at a
        // constant duty cycle regardless of flywheel speed. Hold Y to run them only while the
        // flywheel has settled on its target RPM (Flywheel.atTargetSpeed()) — both pause/resume
        // together each cycle so a second ball doesn't press against the first while the loader
        // is paused waiting for the flywheel to come back up to speed.
        m_driverController.b().whileTrue(
            m_loader.feed(IOConstants.kIndexerFeedPercent)
                .alongWith(m_intake.runIntakeCommand()));
        m_driverController.y().whileTrue(
            m_loader.feedWhen(-IOConstants.kLoaderToFlywheelOutput, m_flywheel::atTargetSpeed)
                .alongWith(m_intake.runIntakeWhen(m_flywheel::atTargetSpeed)));

        // One-button shoot: hold A to spin the flywheel up to the 1-meter preset and feed/intake
        // a ball in once it's settled on speed, gated the same way as the Y binding above.
        m_driverController.a().whileTrue(
            m_flywheel.runAtRPM(3500)
                .alongWith(m_loader.feedWhen(-IOConstants.kLoaderToFlywheelOutput, m_flywheel::atTargetSpeed))
                .alongWith(m_intake.runIntakeWhen(m_flywheel::atTargetSpeed)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Autos.driveDistance(m_driveSubsystem);
    }
    
}
