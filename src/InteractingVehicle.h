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

#include <omnetpp.h>
#include "veins/modules/application/ieee80211p/DemoBaseApplLayer.h"
#include "InterVehicleMessage_m.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"


using veins::Coord;
using veins::DemoBaseApplLayer;

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

    /**
     * Finish hook. finish() is called after end of simulation if it
     * terminated without error.
     */
    virtual void finish();

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
     * Handles message generation
     */
    virtual InterVehicleMessage* generateMsg();

    /**
     * Handles message sending
     */
    virtual void sendMsg(InterVehicleMessage* msg);
};

#endif
