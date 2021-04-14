package frc.robot.commands.autonomous;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AlignWithGyroCommand;
import frc.robot.commands.AlignWithLimelightCommand;
import frc.robot.commands.AutomaticShootCommand;
import frc.robot.commands.ChargeAutoCommand;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.common.Odometry;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.*;

public class ShootIntakeShootCommand extends SequentialCommandGroup {
    // shoot 3 balls and move backwards to pick up 3 more and shoot new 3 balls
    private final IntakeSubsystem intakeSubsystem;

    public ShootIntakeShootCommand(
            DriveSubsystem driveSubsystem,
            Odometry odometry,
            Limelight limelight,
            AHRS navx,
            IntakeSubsystem intakeSubsystem,
            ShooterSubsystem shooterSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        TrajectoryConfig reverseConfig =
                new TrajectoryConfig(1, Constants.MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE))
                        .setReversed(true);
        
        TrajectoryConfig config =
                new TrajectoryConfig(1, Constants.MAX_GENERATION_ACCELERATION)
                        .setKinematics(DriveSubsystem.KINEMATICS)
                        .addConstraint(
                                new DifferentialDriveVoltageConstraint(
                                        DriveSubsystem.FEED_FORWARD,
                                        DriveSubsystem.KINEMATICS,
                                        Constants.MAX_GENERATION_VOLTAGE));

        Trajectory intakeThreeBalls = TrajectoryGenerator.generateTrajectory(
                new Pose2d(3.1, 0, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(7.75, 0, Rotation2d.fromDegrees(180)),
                reverseConfig);

        Trajectory moveForwards = TrajectoryGenerator.generateTrajectory(
                new Pose2d(7.75, 0, Rotation2d.fromDegrees(180)),
                List.of(),
                new Pose2d(6.5, 0, Rotation2d.fromDegrees(190)),
                config);

        addCommands(
                new InstantCommand(() -> navx.reset()),
                new AlignWithLimelightCommand(limelight, driveSubsystem),
                new ParallelRaceGroup(
                        new AutomaticShootCommand(4700, 3, shooterSubsystem).withTimeout(5),
                        new RunCommand(() -> this.intakeSubsystem.spin(-7, 0), this.intakeSubsystem)),
                new InstantCommand(() -> this.intakeSubsystem.spin(0, 0), this.intakeSubsystem),
                new AlignWithGyroCommand(navx, driveSubsystem, 0),
                new InstantCommand(this.intakeSubsystem::extend, this.intakeSubsystem),
                new ParallelRaceGroup(
                        new FollowTrajectoryCommand(intakeThreeBalls, odometry, driveSubsystem),
                        new RunCommand(() -> this.intakeSubsystem.spin(-7.5, -5.2), this.intakeSubsystem)),
                new FollowTrajectoryCommand(moveForwards, odometry, driveSubsystem, false),
                new AlignWithLimelightCommand(limelight, driveSubsystem),
                new ParallelRaceGroup(
                        new AutomaticShootCommand(4900, 3, shooterSubsystem).withTimeout(5),
                        new RunCommand(() -> this.intakeSubsystem.spin(-7.5, -5), this.intakeSubsystem)));
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSubsystem.spin(0, 0);
    }
}
