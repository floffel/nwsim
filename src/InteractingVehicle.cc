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

// TODO:
// Annahme: Es bremst immer das langsamere auto, wenn beide gleichschnell sind, das, was eine höhere id hat
// TODO: nur dann eine Nachricht broadcasten, wenn man kurz vor der kreuzung steht
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

            // getLaneIds()
            /*
            {
                auto* road = new veins::TraCICommandInterface::Road(traci, traciVehicle->getRoadId());
                auto* move = new veins::Move();
                //move->setDirectionByTarget();
                move->setSpeed(road->getMeanSpeed());
            }
            */

            /***
             * Offene Fragen:
             * - TraCICommandInterface::Vehicle::stopAt() soll ein radius mitgegeben werden, der nie benutzt wird. Wieso soll dieser mitgegeben werden?
             */

        }

        psInterval = par("psInterval");
        EV << "psInterval: " << psInterval << std::endl;
        sendPSEvt = new cMessage("PSEvt", SEND_PS_EVT);
        sendMeetingAnnouncementEvt = new cMessage("MeetingAnnouncementEvt", SEND_MEETING_ANNOUNCEMENT_EVT);
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
    EV << "new message" << std::endl;

    if(mobility->getSpeed() == 0) // can't do anything with no speed
        return;

    // put handling of messages from other nodes here

    if(InterVehicleMessage *ivmsg = check_and_cast<InterVehicleMessage *>(msg)) {
        EV << "[" << getParentModule()->getFullName() << "]" << "Got a new InterVehicleMessage from " << ivmsg->getName() << "with position " << ivmsg->getPosition() << ", Speed: " << ivmsg->getSpeed() << std::endl;

        std::string enemy_name = ivmsg->getName();
        veins::Coord enemy_last_pos = enemys_last_position[ivmsg->getName()];
        veins::Coord enemy_current_pos = ivmsg->getPosition();
        veins::Coord enemy_vec = enemy_current_pos - enemy_last_pos;
        double enemy_current_speed = ivmsg->getSpeed();

        veins::Coord my_current_pos = mobility->getPositionAt(simTime());
        veins::Coord my_vec = my_current_pos - my_last_position;
        double my_current_speed = mobility->getSpeed();
        veins::Coord my_next_point; // this is not only the next point. It is the boundary and the potential meeting point, too


        if(enemy_last_pos == veins::Coord().ZERO || // no last position
           my_current_speed == 0 || enemy_current_speed == 0 || // would give false positives/negatives
           .1 > std::abs(my_vec.twoDimensionalCrossProduct(enemy_vec)) // parallel
           ) {
            enemys_last_position[ivmsg->getName()] = enemy_current_pos;
            my_last_position = my_current_pos;
            return;
        }


        // 4 future me, implement this futher: Annahme:
        // Er fährt die lane shapes ab, in der reihenfolge. dann kann man mit getLanePosition() schauen, vor welchem er ist.
        // starten ab index 1 und
        //   schauen, ob die distanz vom ersten zum nächsten größer ist,
        //      wenn ja, ist es der punkt
        //      wenn nein, dann iterieren

        // zusatz: beim schauen mit den junction coordinaten vergleichen, sonst iterieren, um so nur die junctions zu bekommen
        {
            double already_driven = traciVehicle->getLanePosition();
            auto lane = veins::TraCICommandInterface::Lane(traci, traciVehicle->getLaneId());
            std::list<veins::Coord> shape = lane.getShape();

            veins::Coord start = shape.front();
            shape.pop_front();
            for(auto c : shape) {
                if(traci->getDistance(start, c, true) > already_driven) {
                    EV << "found nextpoint! " << c << std::endl;
                    my_next_point = c;
                    break;
                }
            }

        }


        { // calculate the meeting time

            // cant use true (for "real" distance, dunno why, it will not find a route all the time)
            double dist_my = traci->getDistance(my_current_pos, my_next_point, false);
            double dist_enemy = traci->getDistance(enemy_current_pos, my_next_point, false);

            EV << "my distance: " << dist_my << std::endl;
            EV << "enemy distance: " << dist_enemy << std::endl;

            double enemy_time = dist_enemy / enemy_current_speed;
            double my_time = dist_my / my_current_speed;

            double time_diff = enemy_time - my_time;

            if(std::abs(time_diff) < critical_meeting_duration) {
                EV << "planning meeting in" << my_time << std::endl;

                meetings[enemy_name] = simTime() + my_time;
                announceNextMeeting();
            }

        }
    }
}

simtime_t InteractingVehicle::getNextMeetingTime() {
    simtime_t ealiest_meeting = simtime_t::getMaxTime();

    // TODO: imit the name as we dont need it (?)
    for(const auto& [name, time] : meetings) {
        if (time < ealiest_meeting && time >= simTime()) {
            ealiest_meeting = time;
        }
    }

    return ealiest_meeting;
}


void InteractingVehicle::announceNextMeeting() {
    simtime_t time = getNextMeetingTime();

    if(sendMeetingAnnouncementEvt->isScheduled())
        cancelEvent(sendMeetingAnnouncementEvt);

    if(time == simtime_t::getMaxTime())
        return;

    if(simTime() - time > 3) { // send it as a warning message, if we have time
        EV << "+++++ scheduling meeting @" << time - 3 << std::endl;
        scheduleAt(time - 3, sendMeetingAnnouncementEvt);
    }
    else { // send as an automatic break message
        EV << "+++++ scheduling meeting @" << time << std::endl;
        scheduleAt(time, sendMeetingAnnouncementEvt);
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
            //sendDown(generateMsg());
            // todo: we have to send the psEvtMessage down, not a new one

            InterVehicleMessage* wiv = new InterVehicleMessage(getParentModule()->getFullName());

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
        case SEND_MEETING_ANNOUNCEMENT_EVT: {
            simtime_t time = getNextMeetingTime();

            if(time == simtime_t::getMaxTime())
                break;

            if(simTime() - time <= 3) {
                bubble("WARNING!");
                findHost()->bubble("Warning! Crash incoming");
                EV << "<<< BUBBELING!" << getParentModule()->getFullName() << std::endl;
            }

                // trigger warning
            //else // send as an automatic break message
            //    scheduleAt(time, sendMeetingAnnouncementEvt);

            // might meet another car...
        }
    }

}
