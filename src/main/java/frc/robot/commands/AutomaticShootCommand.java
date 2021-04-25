package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ShooterSubsystem;

public class AutomaticShootCommand extends CommandBase {
    private final ShooterSubsystem shooterSubsystem;
    private final double targetVel;
    private int ballsLeftToShoot;
    private boolean shot = false;
    private long shotTime = 0;

    public AutomaticShootCommand(double targetVel, int ballsLeft, ShooterSubsystem shooterSubsystem) {
        this.targetVel = targetVel;
        this.ballsLeftToShoot = ballsLeft;
        this.shooterSubsystem = shooterSubsystem;

        addRequirements(shooterSubsystem);
    }

    public void end(boolean interrupted) {
        // shooterSubsystem.shootVoltage(0);
        shooterSubsystem.lowerPiston();
        System.out.println("Ending with: " + ballsLeftToShoot);
    }

    public boolean isFinished() {
        return false;
    }

    public void execute() {
        if (targetVel > -1) {
            shooterSubsystem.shootVelocity(targetVel);
        }
        System.out.println("IS BALL READY: " + shooterSubsystem.isBallReady());
        if (!shot && (shooterSubsystem.isAtTargetSpeed() || targetVel < 0)) {
            shooterSubsystem.activatePiston();
            shot = true;
            shotTime = System.currentTimeMillis();
        } else if (shot && System.currentTimeMillis() - shotTime < 1500) {
            if (System.currentTimeMillis() - shotTime > 500) {
                shooterSubsystem.lowerPiston();
            }
        } else if (shot) {
            shot = false;
        }
    }
}
