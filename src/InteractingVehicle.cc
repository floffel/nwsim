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
    //DemoBaseApplLayer::~DemoBaseApplLayer();

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
        my_last_position = mobility->getPositionAt(simTime());

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
    if(InterVehicleMessage *ivmsg = check_and_cast<InterVehicleMessage *>(msg)) {
        EV << "[" << getParentModule()->getFullName() << "]" << "Got a new InterVehicleMessage from " << ivmsg->getName() << "with position " << ivmsg->getPosition() << ", Speed: " << ivmsg->getSpeed() << std::endl;

        std::string enemy_name = ivmsg->getName();
        veins::Coord enemy_last_pos = enemys_last_position[ivmsg->getName()];
        veins::Coord enemy_current_pos = ivmsg->getPosition();
        veins::Coord enemy_vec = enemy_current_pos - enemy_last_pos;
        double enemy_current_speed = ivmsg->getSpeed();
        if(enemy_current_speed == 0)
            enemy_current_speed = 50; // for more safety, pretend we are moving, TODO: use the road mean speed

        veins::Coord my_current_pos = mobility->getPositionAt(simTime());
        veins::Coord my_vec = my_current_pos - my_last_position;
        double my_current_speed = mobility->getSpeed();
        if(my_current_speed == 0)
            my_current_speed = 50; // for more safety, pretend we are moving

        veins::Coord meeting_point;

        meetings.erase(enemy_name);

        // cancelation requirements
        if(enemy_last_pos == veins::Coord().ZERO || // no last position
           veins::math::almost_equal(std::abs(my_vec.twoDimensionalCrossProduct(enemy_vec)), 0.0) // parallel
           ) {
            enemys_last_position[ivmsg->getName()] = enemy_current_pos;
            my_last_position = my_current_pos;
            announceNextMeeting(); // could be, that the enemy was deleted, so we probably have to announce a new meeting
            return;
        }

        EV << "not the same, cross product gives: " << my_vec.twoDimensionalCrossProduct(enemy_vec) << std::endl;

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

            announceNextMeeting();
        }
    }

    delete(msg);
}

std::pair<std::string, simtime_t> InteractingVehicle::getNextMeetingTime() {
    std::pair<std::string, simtime_t> ealiest_meeting = { "", simtime_t::getMaxTime() };

    for(const auto& [name, time] : meetings)
        if (time < ealiest_meeting.second && time >= simTime())
            ealiest_meeting = { name, time };

    return ealiest_meeting;
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

    if(simTime() > schedulingWarnTime && simTime() < time) // should have warned earlier
        scheduleAt(simTime(), sendMWEvt);
    else if(simTime() < schedulingWarnTime) // warn correctly
        scheduleAt(schedulingWarnTime, sendMWEvt);

    bool from_left = false; // only the cars coming from left have to break
    {
        veins::Coord my_current_pos = mobility->getPositionAt(simTime());
        double d = ((enemys_last_position[next_meeting.first] - my_last_position).twoDimensionalCrossProduct(my_current_pos - my_last_position))/(my_current_pos - my_last_position).length();
        EV << "d is: " << d << std::endl;

        from_left = d < 0;
    }

    if(from_left && simTime() > schedulingBreakTime && simTime() < time) { // should have breaked earlier
        // TODO: this three lines are doubled... bad coding style i guess, fix it...
        if(sendDriveAgainEvt->isScheduled())
            cancelEvent(sendDriveAgainEvt);
        scheduleAt(simTime() + breakDuration, sendDriveAgainEvt);

        scheduleAt(simTime(), sendMBEvt);
    }
    else if(from_left && simTime() < schedulingBreakTime) { // warn correctly
        if(sendDriveAgainEvt->isScheduled())
            cancelEvent(sendDriveAgainEvt);
        scheduleAt(schedulingBreakTime + breakDuration, sendDriveAgainEvt);

        scheduleAt(schedulingBreakTime, sendMBEvt);
    }

    //cancelEvent(sendDriveAgainEvt);

    //EV << "+++++ scheduling meeting @" << schedulingTime << std::endl;
    //scheduleAt(schedulingTime, sendMeetingAnnouncementEvt);
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
            //sendDown(generateMsg());
            // todo: we have to send the psEvtMessage down, not a new one

            wiv = new InterVehicleMessage(getParentModule()->getFullName());

            auto position = mobility->getPositionAt(simTime()); // DemoBaseApplLayer::curPosition
            auto speed = mobility->getSpeed(); // DemoBaseApplLayer::curSpeed

            //auto position = DemoBaseApplLayer::curPosition;
            //auto speed = DemoBaseApplLayer::curSpeed;

            wiv->setPosition(position);
            wiv->setSpeed(speed);
            wiv->setChannelNumber(static_cast<int>(veins::Channel::cch));

            sendDown(wiv);

            scheduleAt(simTime() + psInterval, sendPSEvt);
            break;
        }
        case SEND_MW_EVT: { // Warning triggered
            findHost()->bubble("Warning!");
            getParentModule()->getDisplayString().setTagArg("i", 1, "red");
            break;
        }
        case SEND_MB_EVT: { // Breaking triggered
            auto next_meeting = getNextMeetingTime();

            if(next_meeting.second == simtime_t::getMaxTime()) // meeting was deleted
                break;

            EV << getParentModule()->getFullName() << ": Breaking in favor of enemy car " << next_meeting.first << std::endl;
            findHost()->bubble("Breaking!");
            traciVehicle->setSpeed(0);
            getParentModule()->getDisplayString().setTagArg("i", 1, "grey");

            meetings.erase(next_meeting.first); // meeting over, so we can remove it from the list
            //announceNextMeeting(); // endlosschleife

            break;
        }
        case SEND_DRIVE_AGAIN_EVT: {
            announceNextMeeting(); // checks, when the next meeting is planned and if we really are allowed to drive
            // TODO: check if the way is clear (e.g. calling getNextMeetingTime)
            // TODO: get the main speed for the track?
            findHost()->bubble("Driving");
            getParentModule()->getDisplayString().setTagArg("i", 1, "green");
            traciVehicle->setSpeed(50);
            announceNextMeeting();
            break;
        }
    }

}
