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
        SEND_MTW_EVT = 998,
        SEND_MTC_EVT = 997,
        SEND_MB_EVT = 996,
        SEND_DRIVE_AGAIN_EVT = 995,
        SEND_SD_EVT = 994,
        SEND_USD_EVT = 993
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

    virtual void refreshDisplay() const override;

    virtual bool isFromLeft(std::string name);

    virtual void continueDriving();

    virtual double getAverageSpeed(std::vector<std::tuple<veins::Coord, double, simtime_t>> hist);

    virtual double getAverageAcceleration(std::vector<std::tuple<veins::Coord, double, simtime_t>> hist);

    virtual void addDrivingHistory();
    virtual void setMeetingAvoidingSpeed();

    virtual std::pair<std::string, simtime_t> getNextMeetingFromLeft();

    std::map<std::string, std::vector<std::tuple<veins::Coord, double, simtime_t>>> enemysDrivingHistory;
    std::vector<std::tuple<veins::Coord, double, simtime_t>> myDrivingHistory;

    // histories to only send warnings one time
    // will be cleared once a car continues driving after a meeting
    // TODO: those will never be emtied for the car that comes from the right, fix this, future me!... :)
    std::map<std::string, bool> timeWarnHistory;
    std::map<std::string, bool> colissionWarnHistory;
    std::map<std::string, bool> breakWarnHistory;

    // to catch the last send position and not resending it
    veins::Coord last_sent;

    // holds the (possible) meetings
    // bool holds the "am I the last one attending the meeting" switch
    // double holds the speed we would need to drive to not attend the meeting
    std::map<std::string, std::tuple<simtime_t, bool, double>> meetings;

    veins::TraCIMobility* mobility;
    veins::TraCICommandInterface* traci;
    veins::TraCICommandInterface::Vehicle* traciVehicle;

    /* Duration a car has before a meeting occurs when the time warning shall appear */
    simtime_t meetTimeWarnBefore;
    /* Duration a car has before a meeting occurs when the collision warning shall appear */
    simtime_t meetCollissionWarnBefore;
    /* Duration a car is before to meet when a break shall be initiated */
    simtime_t meetBreakBefore;

    /* Duration (in seconds) when a meeting will occur, if two cars are at the same point */
    simtime_t criticalMeetingDuration;
    /* save initial max Speed set by TraCI for slowing down */
    double maxSpeed;

    /* Duration before a car shall resume after a meeting */
    simtime_t breakDuration;
    /* Interval for PS Messages*/
    simtime_t psInterval;

    /* messages for periodic events, namely the position/speed update transmissions */
    cMessage* sendPSEvt;
    /* messages for MeetingTimeWarning */
    cMessage* sendMTWEvt;
    /* messages for MeetingColissionWarning */
    cMessage* sendMCWEvt;
    /* messages for MeetingBreaking */
    cMessage* sendMBEvt;
    /* messages for announcing (potential) meetings and for triggering actions */
    cMessage* sendDriveAgainEvt;
};

#endif
