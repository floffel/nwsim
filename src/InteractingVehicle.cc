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

#include <vector>
#include "InteractingVehicle.h"

Define_Module(InteractingVehicle);

InteractingVehicle::~InteractingVehicle()
{
    cancelAndDelete(sendPSEvt);
    cancelAndDelete(sendDriveAgainEvt);
    cancelAndDelete(sendMWEvt);
    cancelAndDelete(sendMBEvt);
}

void InteractingVehicle::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        // Initializing members and pointers of your application goes here

        // initialize pointers to other modules
        if (veins::FindModule<veins::TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = veins::TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface(); // hier gib es getRouteIds und getPlannedRouteIds
            traciVehicle = mobility->getVehicleCommandInterface(); // hier gibt es getLanePosition()

            /***
             * Offene Fragen:
             * - TraCICommandInterface::Vehicle::stopAt() soll ein radius mitgegeben werden, der nie benutzt wird. Wieso soll dieser mitgegeben werden?
             */

        }

        // get parameters
        breakDuration = par("breakDuration");
        meetWarnBefore = par("meetWarnBefore");
        meetBreakBefore = par("meetBreakBefore");
        criticalMeetingDuration = par("criticalMeetingDuration");
        psInterval = par("psInterval");
        sendPSEvt = new cMessage("PSEvt", SEND_PS_EVT);
        sendDriveAgainEvt = new cMessage("DriveAgainEvt", SEND_DRIVE_AGAIN_EVT);

        sendMWEvt = new cMessage("MeetingWarningEvt", SEND_MW_EVT);
        sendMBEvt = new cMessage("MeetingBreakingEvt", SEND_MB_EVT);
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
        // initialize the start position
        //my_last_position = mobility->getPositionAt(simTime());

        simtime_t randomOffset = dblrand() * psInterval;
        scheduleAt(simTime() + randomOffset, sendPSEvt);
        // globalStats = FindModule<GlobalStatistics*>::findSubModule(getParentModule()->getParentModule());
    }
}

void InteractingVehicle::handleMessage(cMessage* msg)
{

    // handle self messages (e.g. timers) in a separate methods
    if (msg->isSelfMessage()) {
        handleSelfMsg(msg);
        return;
    }

    // put handling of messages from other nodes here
    if(InterVehicleMessage *ivmsg = check_and_cast<InterVehicleMessage *>(msg))
        onInterVehicleMessage(ivmsg);

    delete(msg);
}

double InteractingVehicle::getAverageSpeed(std::vector<std::pair<veins::Coord, double>> hist) {
    int speed = 0;
    for(auto entry : hist)
        speed += (entry.second / entry.first.length());
    return speed / hist.size();
}

void InteractingVehicle::calculateMeetings() {
    if(my_last_position.size() < 2)
        return;

    veins::Coord my_current_pos = my_last_position[my_last_position.size()-1].first;
    veins::Coord my_vec =  my_last_position[my_last_position.size()-1].first -  my_last_position[my_last_position.size()-2].first;
    double my_current_speed = getAverageSpeed(my_last_position);

    meetings.clear();

    for(const auto& [name, history] : enemys_last_position) {
        if(enemys_last_position.size() < 2)
            continue;

        veins::Coord meeting_point;

        std::string enemy_name = name;
        veins::Coord enemy_current_pos = history[history.size()-1].first;
        veins::Coord enemy_vec = history[history.size()-1].first -  history[history.size()-2].first;
        double enemy_current_speed = getAverageSpeed(history);

        if(std::abs(my_vec.twoDimensionalCrossProduct(enemy_vec)) == 0.0) //parallel
            continue;

        { // calculate intersection, see https://stackoverflow.com/a/565282
            double my_n = (enemy_current_pos - my_current_pos).twoDimensionalCrossProduct(enemy_vec) / my_vec.twoDimensionalCrossProduct(enemy_vec);
            double enemy_n = (my_current_pos - enemy_current_pos).twoDimensionalCrossProduct(my_vec) / enemy_vec.twoDimensionalCrossProduct(my_vec);

            if(my_n < 0 || enemy_n < 0) // seems we'd have to drive backwards to reach that point...
                return;

            EV << "my position for the meeting is " << my_current_pos + (my_vec * my_n) << std::endl
               << "  enemys position is " << my_current_pos + (my_vec * my_n) << std::endl
               << "my n is: " << my_n << "enemys n is: " << enemy_n << std::endl;

            meeting_point = my_current_pos + (my_vec * my_n);
        }

        { // calculate the meeting time
            // cant use true, maybe because its not perfectly "on their route"? Todo: fix this..
            double dist_my = traci->getDistance(my_current_pos, meeting_point, false);
            double dist_enemy = traci->getDistance(enemy_current_pos, meeting_point, false);

            double enemy_time = dist_enemy / enemy_current_speed;
            double my_time = dist_my / my_current_speed;

            double time_diff = enemy_time - my_time;

            if(std::abs(time_diff) < criticalMeetingDuration) {
                EV << "planning meeting in" << my_time << std::endl;
                meetings[enemy_name] = simTime() + my_time;
            }
        }
    }

    announceNextMeeting();
}

void InteractingVehicle::onInterVehicleMessage(InterVehicleMessage *ivmsg) {
    EV << "[" << getParentModule()->getFullName() << "]" << "Got a new InterVehicleMessage from " << ivmsg->getName() << "with position " << ivmsg->getPosition() << ", Speed: " << ivmsg->getSpeed() << std::endl;

    enemys_last_position[ivmsg->getName()].push_back({ ivmsg->getPosition(), ivmsg->getSpeed() });

    calculateMeetings();
}

void InteractingVehicle::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);

    EV << "handling position update event" << std::endl;
    my_last_position.push_back({ mobility->getPositionAt(simTime()), mobility->getSpeed() });

    calculateMeetings();
}

std::pair<std::string, simtime_t> InteractingVehicle::getNextMeetingTime(int offset) {
    std::pair<std::string, simtime_t> ealiest_meeting = { "", simtime_t::getMaxTime() };

    for(const auto& [name, time] : meetings) {
        if (time < ealiest_meeting.second && time >= simTime() && offset == 0)
            ealiest_meeting = { name, time };
        else
            offset--;
    }

    return ealiest_meeting;
}

// returns true if from left, in all other cases (even in failures) returns false
bool InteractingVehicle::fromLeft(std::string name) {
    auto enemys_hist = enemys_last_position[name];
    if(enemys_hist.size() < 2 || my_last_position.size() < 2)
        return false;

    veins::Coord my_current_pos = my_last_position[my_last_position.size()-1].first;
    veins::Coord my_last_pos = my_last_position[my_last_position.size()-2].first;
    veins::Coord enemys_current_pos = enemys_hist[enemys_hist.size()-1].first;

    double d = ((enemys_current_pos - my_last_pos).twoDimensionalCrossProduct(my_current_pos - my_last_pos))/(my_current_pos - my_last_pos).length();
    EV << "d is: " << d << std::endl;

    return d < 0;
}

// get the first meeting where we would be the car from the left

std::pair<std::string, simtime_t> InteractingVehicle::getNextMeetingFromLeft() {
    auto next_meeting = getNextMeetingTime();
    int off = 1;

    while(fromLeft(next_meeting.first) == false &&           // not from left
          next_meeting.second == simtime_t::getMaxTime()) {  // nothing more to do, all iterated
        next_meeting = getNextMeetingTime(off);

        off++;
    }

    return next_meeting;
}

void InteractingVehicle::announceNextMeeting() {
    auto next_meeting = getNextMeetingTime();
    simtime_t time = next_meeting.second;

    if(sendMWEvt->isScheduled())
        cancelEvent(sendMWEvt);
    if(sendMBEvt->isScheduled())
        cancelEvent(sendMBEvt);

    if(time == simtime_t::getMaxTime())
        return;

    simtime_t schedulingWarnTime = time - meetWarnBefore;
    simtime_t schedulingBreakTime = time - meetBreakBefore;

    if(simTime() >= schedulingWarnTime && simTime() <= time) // should have warned earlier
        scheduleAt(simTime(), sendMWEvt);
    else if(simTime() < schedulingWarnTime) // warn correctly
        scheduleAt(schedulingWarnTime, sendMWEvt);

    next_meeting = getNextMeetingFromLeft();
    if(next_meeting.second == simtime_t::getMaxTime())
        return; // no meeting found

    if(simTime() >= schedulingBreakTime && simTime() <= time) { // should have breaked earlier
        // TODO: this three lines are doubled... bad coding style i guess, fix it...
        if(sendDriveAgainEvt->isScheduled())
            cancelEvent(sendDriveAgainEvt);
        scheduleAt(simTime() + breakDuration, sendDriveAgainEvt);

        scheduleAt(simTime(), sendMBEvt);
    }
    else if(simTime() < schedulingBreakTime) { // warn correctly
        if(sendDriveAgainEvt->isScheduled())
            cancelEvent(sendDriveAgainEvt);
        scheduleAt(schedulingBreakTime + breakDuration, sendDriveAgainEvt);

        scheduleAt(schedulingBreakTime, sendMBEvt);
    } else {
        if(sendDriveAgainEvt->isScheduled())
            cancelEvent(sendDriveAgainEvt);
    }
}

void InteractingVehicle::finish()
{
    DemoBaseApplLayer::finish();
    // maybe you want to record some scalars?
}

void InteractingVehicle::handleSelfMsg(cMessage* msg)
{
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
    DemoBaseApplLayer::handleSelfMsg(msg);

    switch (msg->getKind()) {
        case SEND_PS_EVT: {
            auto wiv = new InterVehicleMessage(getParentModule()->getFullName());

            auto position = mobility->getPositionAt(simTime());
            auto speed = mobility->getSpeed();

            wiv->setPosition(position);
            wiv->setSpeed(speed);
            wiv->setChannelNumber(static_cast<int>(veins::Channel::cch));

            sendDown(wiv);

            scheduleAt(simTime() + psInterval, sendPSEvt);
            break;
        }
        case SEND_MW_EVT: { // Warning triggered
            auto next_meeting = getNextMeetingTime();

            if(next_meeting.second == simtime_t::getMaxTime()) // meeting was deleted
                break;

            findHost()->bubble("Warning!");
            getParentModule()->getDisplayString().setTagArg("i", 1, "red");
            break;
        }
        case SEND_MB_EVT: { // Breaking triggered
            auto next_meeting = getNextMeetingFromLeft();
            if(next_meeting.second == simtime_t::getMaxTime())
                break; // no meeting found

            EV << getParentModule()->getFullName() << ": Breaking in favor of enemy car " << next_meeting.first << std::endl;
            findHost()->bubble("Breaking!");
            traciVehicle->setSpeed(0);
            getParentModule()->getDisplayString().setTagArg("i", 1, "grey");

            meetings.erase(next_meeting.first); // meeting over, so we can remove it from the list
            announceNextMeeting();

            break;
        }
        case SEND_DRIVE_AGAIN_EVT: {
            announceNextMeeting(); // checks, when the next meeting is planned and if we really are allowed to drive
            findHost()->bubble("Driving");
            getParentModule()->getDisplayString().setTagArg("i", 1, "green");
            traciVehicle->setSpeed(50);
            break;
        }
    }

}
