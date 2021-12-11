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

#include <vector>
#include <tuple>
#include <map>
#include <cmath>
#include <omnetpp.h>
#include "veins/base/utils/FWMath.h"
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
    ~InteractingVehicle() override;

    /**
     * Finish hook. finish() is called after end of simulation if it
     * terminated without error.
     */
    virtual void finish();

    // we have to take different ids becourse of DemoBaseApplLayer, so we'll go from 999 to 0
    enum InteractingVehicleMessageKinds {
        SEND_PS_EVT = 999,
        SEND_MW_EVT = 998,
        SEND_MB_EVT = 997,
        SEND_DRIVE_AGAIN_EVT = 996
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
     * Gets the next time a meeting could take place
     */
    virtual std::pair<std::string, simtime_t> getNextMeetingTime();

    /**
     * Announces a selfMessage for handling the meeting
     */
    virtual void announceNextMeeting();

    std::map<std::string, veins::Coord> enemys_last_position;
    veins::Coord my_last_position;

    // hold the possible meeting messages to trigger a warning etc.
    //std::map<std::string, cMessage*> meeting_messages;
    // bool holds the "was warned already", 3sec before
    std::map<std::string, simtime_t> meetings;

    veins::TraCIMobility* mobility;
    veins::TraCICommandInterface* traci;
    veins::TraCICommandInterface::Vehicle* traciVehicle;

    /* Duration a car has before to meet when a warning shall appear */
    simtime_t meetWarnBefore;
    /* Duration a car is before to meet when a break shall be initiated */
    simtime_t meetBreakBefore;

    double criticalMeetingDuration;

    /* Duration before a car shall resume after a meeting */
    simtime_t breakDuration;
    /* Interval for PS Messages*/
    simtime_t psInterval;

    /* messages for periodic events, namely the position/speed update transmissions */
    cMessage* sendPSEvt;
    /* messages for MeetingWarning */
    cMessage* sendMWEvt;
    /* messages for MeetingBreaking */
    cMessage* sendMBEvt;
    /* messages for announcing (potential) meetings and for triggering actions */
    cMessage* sendDriveAgainEvt;

    InterVehicleMessage* wiv;
};

#endif
