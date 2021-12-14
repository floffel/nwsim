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
#include <math.h>
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
    virtual void finish() override;

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
    virtual void initialize(int stage) override;

    /**
      * @brief Contains the module's message handling function.
      *
      * This is called both when a self message or a message from
      * another node reaches this node.
      */
    virtual void handleMessage(cMessage* msg) override;

    /** @brief handles self messages, such as timers. */
    virtual void handleSelfMsg(cMessage* msg) override;

    /** @brief gets the next time a meeting could take place */
    virtual std::pair<std::string, simtime_t> getNextMeetingTime(int offset=0);

    /** @brief announces self message for handling the next meeting, if any */
    virtual void announceNextMeeting();

    /** @brief handle messages from other vehicles */
    virtual void onInterVehicleMessage(InterVehicleMessage *ivmsg);

    /** @brief (re)calculate meetings */
    virtual void calculateMeetings();

    virtual void handlePositionUpdate(cObject* obj) override;


    virtual bool isFromLeft(std::string name);

    virtual void continueDriving();

    virtual double getAverageSpeed(std::vector<std::tuple<veins::Coord, double, simtime_t>> hist);

    virtual double getAverageAcceleration(std::vector<std::tuple<veins::Coord, double, simtime_t>> hist);

    virtual std::pair<std::string, simtime_t> getNextMeetingFromLeft();

    std::map<std::string, std::vector<std::tuple<veins::Coord, double, simtime_t>>> enemysDrivingHistory;
    std::vector<std::tuple<veins::Coord, double, simtime_t>> myDrivingHistory;

    // to catch the last send position and not resending it
    veins::Coord last_sent;

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
};

#endif
