// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.IOConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.SignalLogger;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Feeds game pieces from the intake into the flywheel (1x REV NEO / SparkMax, brushless).
 *
 * <p>Runs open-loop (duty cycle), like the intake — the loader has three distinct
 * situations (receiving from the intake, sending back out through the intake, and feeding
 * the flywheel), each with its own speed to keep rollers speeding up in the direction of
 * travel and avoid jams. See {@code kLoaderFromIntakeOutput}, {@code kLoaderToIntakeOutput},
 * and {@code kLoaderToFlywheelOutput} in {@link frc.robot.Constants.IOConstants}.
 */
public class Loader extends SubsystemBase {

    private final SparkMax m_loaderMotor;

    // Last duty cycle commanded to the loader motor, kept only so periodic() can log it for
    // ball-feed testing (e.g. to see on the log when feed()/feedWhen() actually ran the motor).
    private double m_lastCommandedOutput = 0.0;

    /** Creates a new Loader subsystem, configuring the loader's SparkMax. */
    public Loader() {
        m_loaderMotor = new SparkMax(kLoaderMotorID, MotorType.kBrushless);
        SparkMaxConfig loaderConfig = new SparkMaxConfig();
        loaderConfig.smartCurrentLimit(kLoaderMotorCurrentLimit);
        loaderConfig.idleMode(IdleMode.kCoast);
        loaderConfig.inverted(true); // clockwise positive
        m_loaderMotor.configure(loaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /** Shared open-loop helper — runs the loader motor at the given duty cycle while active. */
    private Command runLoaderCommand(double output) {
        return run(() -> {
            m_loaderMotor.set(output);
            m_lastCommandedOutput = output;
        }).finallyDo(() -> stop());
    }

    /** Immediately stops the loader motor. */
    public void stop() {
        m_loaderMotor.stopMotor();
        m_lastCommandedOutput = 0.0;
    }

    /**
     * Command factory for ball-feed testing: runs the loader at a constant duty cycle while held,
     * and stops it when released. Bind as a "hold to feed" button.
     *
     * @param percentOutput open-loop duty cycle ([-1, 1]) to feed at, e.g. kIndexerFeedPercent
     */
    public Command feed(double percentOutput) {
        return runLoaderCommand(percentOutput).withName("loaderFeed");
    }

    /**
     * Command factory for ball-feed testing: like feed(), but only actually drives the motor while
     * {@code condition} is true (e.g. Flywheel::atTargetSpeed), so a ball is only fed in once the
     * flywheel has settled on its target speed. Still stops the motor when released.
     *
     * @param percentOutput open-loop duty cycle ([-1, 1]) to feed at while condition is true
     * @param condition supplier checked every cycle; the motor is stopped whenever it's false
     */
    public Command feedWhen(double percentOutput, BooleanSupplier condition) {
        return run(() -> {
            if (condition.getAsBoolean()) {
                m_loaderMotor.set(percentOutput);
                m_lastCommandedOutput = percentOutput;
            } else {
                stop();
            }
        }).finallyDo(() -> stop()).withName("loaderFeedWhen");
    }

    /**
     * Runs the loader in sync with the intake — pulling a game piece in from the intake by
     * default, or running backward to send a piece back out through a reversed intake (e.g.
     * clearing a jam) when {@code reverse} is true. The two directions use different speeds
     * ({@code kLoaderFromIntakeOutput} / {@code kLoaderToIntakeOutput}) since the roller ahead
     * of the loader is different in each case — see the comment on those constants.
     *
     * @param reverse true to run the loader backward, matching {@code Intake.reverseIntakeCommand()}
     */
    public Command runWithIntakeCommand(boolean reverse) {
        double output = reverse ? -kLoaderToIntakeOutput : kLoaderFromIntakeOutput;
        return runLoaderCommand(output).withName(reverse ? "loaderWithIntakeReverse" : "loaderWithIntake");
    }

    /** Runs the loader forward in sync with the intake. Equivalent to runWithIntakeCommand(false). */
    public Command runWithIntakeCommand() {
        return runWithIntakeCommand(false);
    }

    /** Runs the loader at the speed used while feeding a game piece into the flywheel. */
    public Command runToFlywheelCommand() {
        return runLoaderCommand(-kLoaderToFlywheelOutput).withName("loaderToFlywheel");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Indexer/Output", m_lastCommandedOutput);
        SignalLogger.writeDouble("Indexer/Output", m_lastCommandedOutput);
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}
