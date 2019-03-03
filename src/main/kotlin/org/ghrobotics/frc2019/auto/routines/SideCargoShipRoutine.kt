//package org.ghrobotics.frc2019.auto.routines
//
//import org.ghrobotics.frc2019.auto.Autonomous
//import org.ghrobotics.frc2019.auto.paths.TrajectoryFactory
//import org.ghrobotics.frc2019.subsystems.Superstructure
//import org.ghrobotics.frc2019.subsystems.arm.ArmSubsystem
//import org.ghrobotics.frc2019.subsystems.arm.ClosedLoopArmCommand
//import org.ghrobotics.frc2019.subsystems.drive.DriveSubsystem
//import org.ghrobotics.frc2019.subsystems.intake.IntakeCargoCommand
//import org.ghrobotics.frc2019.subsystems.intake.IntakeHatchCommand
//import org.ghrobotics.frc2019.subsystems.intake.IntakeSubsystem
//import org.ghrobotics.lib.commands.*
//import org.ghrobotics.lib.mathematics.twodim.trajectory.types.duration
//import org.ghrobotics.lib.mathematics.units.Time
//import org.ghrobotics.lib.mathematics.units.degree
//import org.ghrobotics.lib.mathematics.units.second
//import org.ghrobotics.lib.utils.map
//import org.ghrobotics.lib.utils.withEquals
//
//class SideCargoShipRoutine : AutoRoutine() {
//
//    private val path1 = TrajectoryFactory.sideStartToCargoShipS1
//    private val path2 = Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO)
//        .map(TrajectoryFactory.cargoShipS1ToDepot, TrajectoryFactory.cargoShipS1ToLoadingStation)
//    private val path3 = Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO)
//        .map(TrajectoryFactory.depotToCargoShipS2, TrajectoryFactory.loadingStationToCargoShipS2)
//
//    val pathMirrored = Autonomous.startingPosition.withEquals(Autonomous.StartingPositions.LEFT)
//
//    override val duration: Time
//        get() = path1.duration
//
//    override val routine: FalconCommand
//        get() = sequential {
//
//            +ConditionalCommand(
//                Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
//                IntakeCargoCommand(IntakeSubsystem.Direction.HOLD),
//                IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
//            )
//
//            +parallel {
//                +DriveSubsystem.followTrajectory(path1)
//                +sequential {
//                    +DelayCommand(path1.duration - 3.second)
//                    +ConditionalCommand(
//                        Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
//                        Superstructure.kFrontCargoFromLoadingStation.withTimeout(2.second),
//                        Superstructure.kFrontHatchFromLoadingStation.withTimeout(2.second)
//                    )
//                }
//            }
//
//            +ConditionalCommand(
//                Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
//                IntakeCargoCommand(IntakeSubsystem.Direction.RELEASE),
//                IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
//            ).withTimeout(0.5.second)
//
//
//            +parallel {
//                +DriveSubsystem.followTrajectory(path2, pathMirrored)
//                +sequential {
//                    +DelayCommand(0.2.second)
//                    +parallel {
//                        +ConditionalCommand(
//                            Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO),
//                            sequential {
//                                +ConditionCommand { ArmSubsystem.position > 105.degree }
//                                +IntakeCargoCommand(IntakeSubsystem.Direction.HOLD)
//                            })
//                        +ConditionalCommand(
//                            Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO),
//                            Superstructure.kBackCargoFromLoadingStation.withTimeout(2.second),
//                            Superstructure.kBackHatchFromLoadingStation.withTimeout(2.second)
//                        )
//                    }
//                }
//            }
//
//            +ConditionalCommand(
//                Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.HATCH),
//                IntakeHatchCommand(IntakeSubsystem.Direction.HOLD)
//            )
//
//            +parallel {
//                +DriveSubsystem.followTrajectory(path3)
//                +sequential {
//                    +executeFor(
//                        4.second,
//                        Superstructure.kFrontCargoFromLoadingStation
//                    )
//                    +ConditionalCommand(
//                        Autonomous.cargoShipGamePiece1.withEquals(Autonomous.GamePiece.CARGO),
//                        Superstructure.kFrontCargoFromLoadingStation.withTimeout(2.second),
//                        Superstructure.kFrontHatchFromLoadingStation.withTimeout(2.second)
//                    )
//                }
//            }
//
//            +ConditionalCommand(
//                Autonomous.cargoShipGamePiece2.withEquals(Autonomous.GamePiece.CARGO),
//                IntakeCargoCommand(IntakeSubsystem.Direction.RELEASE),
//                IntakeHatchCommand(IntakeSubsystem.Direction.RELEASE)
//            ).withTimeout(0.5.second)
//        }
//
//}
