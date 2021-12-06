//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
//

#ifndef __INTERACTING_VEHICLE_H_
#define __INTERACTING_VEHICLE_H_

#include <tuple>
#include <vector>
#include <omnetpp.h>
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "InterVehicleMessage_m.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"


using veins::Coord;
using veins::DemoBaseApplLayer;


// would be cool to be a real class, like in opengl
typedef std::tuple<double, double, double> vec3d;
typedef std::tuple<double, double, double> point3d;


/**
 * @brief A class for vehicles in SUMO simulations.
 *
 * Each vehicle in a coupled SUMO simulation is mapped to one
 * object of this class, if you use the line '*.node[*].applType = "InteractingVehicle"'
 * in omnetpp.ini. The name of the vehicle can be got by calling
 * <code>getParentModule()->getFullName()</code>.
 */
class InteractingVehicle : public DemoBaseApplLayer {

public:

    ~InteractingVehicle();

    /**
     * Finish hook. finish() is called after end of simulation if it
     * terminated without error.
     */
    virtual void finish();

    enum InteractingVehicleMessageKinds {
        SEND_PS_EVT
    };

protected:

    /**
     * Multi-stage initialization hook.
     */
    virtual void initialize(int stage);

    /**
      * Contains the module's message handling function.
      * This is called both when a self message or a message from
      * another node reaches this node.
      */
    virtual void handleMessage(cMessage* msg);

    /**
     * Handles self messages, such as timers.
     */
    virtual void handleSelfMsg(cMessage* msg);

    /**
     * Handles message population
     */
    //virtual InterVehicleMessage* populateMsg();

    // Deprecated, too complex:
        // vectors, the car will drive in the future
        // todo: delete the already driven vectors?
        // holds <vector, start_time, end_time>
        //std::vector<std::tuple<vec3d, simtime_t_cref>> drive_positions_time;
        // holds the possible crash/meeting carnames, positions and times
        //std::vector<std::tuple<std::string, vec3d, simtime_t_cref>> potencial_meets;

        // wir haben für die berechnung der position nur die koordinaten und speed,
        // also müssen wir daraus die straße ableiten, wenn möglich (ansonsten den vektor nehmen der gegeben ist)
        // und so die meeting points erstellen
        // da es so nur 2 punkte gibt, müssen wir also checken, ob wir uns treffen, wenn wir den vektor länger machen
        // wenn nicht -> gut, wenn ja -> meeting point

    // Annahme: Es kann sich nur immer an Kreuzungen, e.g. am Ende einer Straße getroffen werden
    // potential_meeting_points holds the end of the street as a crossing and therefore a potential meeting point, and the time it will be there
    // This simulates a perfect world, where a road is ending as soon as a crossing occours
    // in the real world, we would have to determine all road-crossings first
    // as I don't know how far this project will go, it will already be a vector, so not too much additional work would be needed
    std::vector<std::tuple<point3d, simtime_t_cref>> potential_meeting_points;

    // meetings holds <name, coordinates, time> the calculated meetings between two cars which would normally result in a crash
    std::vector<std::tuple<std::string, point3d, simtime_t_cref>> meetings;


    veins::TraCIMobility* mobility;
    veins::TraCICommandInterface* traci;
    veins::TraCICommandInterface::Vehicle* traciVehicle;


    simtime_t psInterval;
    /* messages for periodic events, namly the position/speed update transmissions */
    cMessage* sendPSEvt;
};

#endif
