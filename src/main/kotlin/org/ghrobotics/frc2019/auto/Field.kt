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
    val kHabitatL2RX = 47.00.inch
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
    val kRocketN = Pose2d(212.80.inch, 19.20.inch, (-20).degree)
    val kRocketF = Pose2d(243.20.inch, 19.20.inch, (-160).degree)

    val kRocketBay = Pose2d(228.inch, 27.44.inch, (-90).degree)

    /** LOADING STATION **/
    val kLoadingStation = Pose2d(0.inch, 26.94.inch, 0.degree)

    /** DEPOT **/
    val kDepotBRCorner = Pose2d(47.inch, 78.396.feet, (-25).degree)
}