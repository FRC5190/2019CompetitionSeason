package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.Autonomous
import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.ConditionCommand
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.second
import org.ghrobotics.lib.utils.withEquals

class HatchAndCargoRocketRoutine : AutoRoutine() {
    private val path1 = TrajectoryFactory.sideStartToRocketN
    private val path2 = TrajectoryFactory.rocketNToLoadingStation
    private val path3 = TrajectoryFactory.loadingStationToRocketN
    private val path4 = TrajectoryFactory.rocketNToDepot

    override val duration: Time
        get() = path1.duration + path2.duration + path3.duration + path4.duration

    override val routine
        get() = sequential {

            val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)

            +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

            +parallel {
                +DriveSubsystem.followTrajectory(path1, pathMirrored)
                +sequential {
                    +DelayCommand(path1.duration - 4.0.second)
                    +Superstructure.kFrontHighRocketHatch.withTimeout(4.second)
                }
            }

            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
            +DelayCommand(0.1.second)

            +parallel {
                +DriveSubsystem.followVisionAssistedTrajectory(path2, pathMirrored, 5.feet, 2.feet)
                +sequential {
                    +DelayCommand(0.5.second)
                    +Superstructure.kBackHatchFromLoadingStation
                }
            }

            +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

            // Reset odometry at loading station
//        +parallel {
//            +DelayCommand(0.2.second)
//            +ConditionalCommand(IntakeSubsystem.isHoldingHatch, InstantRunnableCommand {
//                DriveSubsystem.localization.reset(TrajectoryWaypoints.kLoadingStation + Constants.kBackwardIntakeToCenter)
//            })
//        }

            // Place hatch on near rocket
            +parallel {
                // Drive path to far rocket
                +DriveSubsystem.followVisionAssistedTrajectory(path3, pathMirrored, 4.feet, 2.feet)
                +sequential {
                    +executeFor(
                        path3.duration - 2.75.second,
                        Superstructure.kFrontCargoFromLoadingStation
                    )
                    +Superstructure.kFrontMiddleRocketHatch
                }
            }

            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
            +DelayCommand(0.3.second)

            // Pickup cargo from depot
            +parallel {
                // Drive path to loading station
                +DriveSubsystem.followTrajectory(path4, pathMirrored)

                +sequential {
                    +DelayCommand(0.5.second)
                    // Take superstructure to pickup ball
                    +Superstructure.kBackCargoIntake.withTimeout(1.5.second)

                    +sequential {
                        +ConditionCommand { ArmSubsystem.armPosition > 150.degree }
                        +IntakeCargoCommand(IntakeSubsystem.Direction.HOLD)
                    }
                }
            }
        }
}
