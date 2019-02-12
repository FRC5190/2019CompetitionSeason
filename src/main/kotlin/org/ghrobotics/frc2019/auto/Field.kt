package org.ghrobotics.frc2019.auto

import org.ghrobotics.lib.mathematics.twodim.geometry.Pose2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Rectangle2d
import org.ghrobotics.lib.mathematics.twodim.geometry.Translation2d
import org.ghrobotics.lib.mathematics.units.degree
import org.ghrobotics.lib.mathematics.units.feet
import org.ghrobotics.lib.mathematics.units.inch

/**
 * Official Field Element Locations from FRC Field Drawings
 */
object Field {

    /** HABITAT **/
    val kHabitatL2RX = 48.00.inch
    val kHabitatL2BY = 97.00.inch
    val kHabitatL1RX = 95.28.feet

    val kHabitatL1Platform = Rectangle2d(Translation2d(4.feet, 7.feet), Translation2d(8.feet, 20.feet))


    /** CARGO SHIP **/
    val kCargoShipFL = Pose2d(220.25.inch, 172.875.inch, 0.degree)
    val kCargoShipFR = Pose2d(220.25.inch, 151.125.inch, 0.degree)

    val kCargoShipS1 = Pose2d(260.75.inch, 133.13.inch, 90.degree)
    val kCargoShipS2 = Pose2d(282.50.inch, 133.13.inch, 90.degree)
    val kCargoShipS3 = Pose2d(304.25.inch, 133.13.inch, 90.degree)


    /** ROCKET **/
    val kRocketN = Pose2d(213.57.inch, 17.074.inch, (-028.720).degree) // Pose2d(213.57.inch, 18.60.inch, (-032.748).degree)
    val kRocketF = Pose2d(244.69.inch, 17.074.inch, (-151.280).degree) // Pose2d(244.69.inch, 18.60.inch, (-147.252).degree)

    val kRocketBay = Pose2d(229.13.inch, 27.44.inch, (-90).degree)


    /** LOADING STATION **/
    val kLoadingStation = Pose2d(0.inch, 25.72.inch, 0.degree)

    /** DEPOT **/
    val kDepotBRCorner = Pose2d(47.inch, 78.396.feet, (-25).degree)
}