package org.ghrobotics.frc2019.auto.routines

import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
import org.ghrobotics.frc2019.subsystems.Superstructure
import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
import org.ghrobotics.lib.commands.DelayCommand
import org.ghrobotics.lib.commands.parallel
import org.ghrobotics.lib.commands.sequential
import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
import org.ghrobotics.lib.mathematics.units.Time
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.second

class ForwardCargoShipRoutine : AutoRoutine() {

    private val path1 = TrajectoryFactory.centerStartToCargoShipFL
    private val path2 = TrajectoryFactory.cargoShipFLToLoadingStation
    private val path3 = TrajectoryFactory.loadingStationToCargoShipFR

    override val duration: Time
        get() = path1.duration + path2.duration + path3.duration

    override val routine
        get() = sequential {
            // Hold hatch
            +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

            // Put hatch on FL cargo ship
            +parallel {
                +DriveSubsystem.followTrajectory(path1)
                +sequential {
                    +executeFor(
                        path1.duration - 2.second,
                        ClosedLoopArmCommand(30.degree)
                    )
                    +Superstructure.kFrontHatchFromLoadingStation
                }
            }

            // Release hatch
            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
            +DelayCommand(0.1.second)

            // Go to loading station
            +parallel {
                +DriveSubsystem.followTrajectory(path2)
                +sequential {
                    +DelayCommand(0.2.second)
                    +Superstructure.kBackHatchFromLoadingStation
                }
            }

            // Pickup hatch
            +IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)

            // Reset odometry at loading station
//        +parallel {
//            +DelayCommand(0.2.second)
//            +ConditionalCommand(IntakeSubsystem.isHoldingHatch, InstantRunnableCommand {
//                DriveSubsystem.localization.reset(TrajectoryWaypoints.kLoadingStation + Constants.kBackwardIntakeToCenter)
//            })
//        }

            // Go to FR cargo ship
            +parallel {
                +DriveSubsystem.followTrajectory(path3)
                +sequential {
                    +executeFor(
                        path3.duration - 2.75.second,
                        Superstructure.kFrontCargoFromLoadingStation
                    )
                    +Superstructure.kFrontHatchFromLoadingStation
                }
            }

            // Place hatch
            +IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
            +DelayCommand(0.3.second)
        }
}