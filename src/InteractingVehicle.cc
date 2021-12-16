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

#include "InteractingVehicle.h"

Define_Module(InteractingVehicle);

InteractingVehicle::~InteractingVehicle()
{
    cancelAndDelete(sendDriveAgainEvt);
    cancelAndDelete(sendMTWEvt);
    cancelAndDelete(sendMCWEvt);
    cancelAndDelete(sendMBEvt);
    cancelAndDelete(sendPSEvt);
}

void InteractingVehicle::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        // Initializing members and pointers of your application goes here

        // initialize pointers to other modules
        if (veins::FindModule<veins::TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = veins::TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();

            /***
             * Offene Fragen:
             * - TraCICommandInterface::Vehicle::stopAt() soll ein radius mitgegeben werden, der nie benutzt wird. Wieso soll dieser mitgegeben werden?
             */

        }

        // get parameters
        breakDuration = par("breakDuration");
        meetTimeWarnBefore = par("meetTimeWarnBefore");
        meetCollissionWarnBefore = par("meetCollissionWarnBefore");
        meetBreakBefore = par("meetBreakBefore");
        criticalMeetingDuration = par("criticalMeetingDuration");
        psInterval = par("psInterval");

        sendPSEvt = new cMessage("PSEvt", SEND_PS_EVT);
        sendDriveAgainEvt = new cMessage("DriveAgainEvt", SEND_DRIVE_AGAIN_EVT);
        sendMCWEvt = new cMessage("MeetingCollisionWarningEvt", SEND_MTC_EVT);
        sendMTWEvt = new cMessage("MeetingTimeWarningEvt", SEND_MTW_EVT);
        sendMBEvt = new cMessage("MeetingBreakingEvt", SEND_MB_EVT);
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
        // initialize the start position
        //my_last_position = mobility->getPositionAt(simTime());

        simtime_t randomOffset = dblrand();// * psInterval;
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

double InteractingVehicle::getAverageSpeed(std::vector<std::tuple<veins::Coord, double, simtime_t>> hist) {
    double speed = 0;
    for(auto entry : hist)
        speed += (std::get<1>(entry) / std::get<0>(entry).length());
    return speed / hist.size();
}

double InteractingVehicle::getAverageAcceleration(std::vector<std::tuple<veins::Coord, double, simtime_t>> hist) {
    double ret = 0;

    if(hist.size() < 2)
        return ret;

    double length = (std::get<0>(hist[1]) - std::get<0>(hist[0])).length();
    simtime_t time = (std::get<2>(hist[1]) - std::get<2>(hist[0]));
    double acceleration = 2*(length/pow(time.dbl(), 2.0));
    return acceleration;
}

void InteractingVehicle::calculateMeetings() {
    if(myDrivingHistory.size() < 2)
        return;

    veins::Coord my_current_pos = std::get<0>(myDrivingHistory[myDrivingHistory.size()-1]);
    veins::Coord my_last_pos = std::get<0>(myDrivingHistory[myDrivingHistory.size()-2]);

    veins::Coord my_vec =  std::get<0>(myDrivingHistory[myDrivingHistory.size()-1]) - my_last_pos;
    my_vec = (my_vec/my_vec.length())*100;

    //double my_current_speed = getAverageSpeed(my_last_position);
    double my_current_speed = std::get<1>(myDrivingHistory[myDrivingHistory.size()-1]);
    // some form of pid controller
    //double my_current_speed = myDrivingHistory[myDrivingHistory.size()-1].second*.95 + getAverageSpeed(myDrivingHistory)*.05;
    //double my_current_speed = myDrivingHistory[myDrivingHistory.size()-1].second + getAverageAcceleration(myDrivingHistory);

    meetings.clear();

    for(const auto& [name, history] : enemysDrivingHistory) {
        EV << "[" << getParentModule()->getFullName() << "]"
                << "calculating if we meet with " << name << std::endl;

        if(history.size() < 2) {
            EV << "  cancel: too less information about enemys history" << name << std::endl;
            continue;
        }

        veins::Coord meeting_point;

        std::string enemy_name = name;
        veins::Coord enemy_current_pos = std::get<0>(history[history.size()-1]);
        veins::Coord enemy_last_pos = std::get<0>(history[history.size()-2]);
        veins::Coord enemy_vec = (std::get<0>(history[history.size()-1]) - enemy_last_pos);

        enemy_vec = (enemy_vec/enemy_vec.length())*100;
        //double enemy_current_speed = getAverageSpeed(history);
        double enemy_current_speed = std::get<1>(history[history.size()-1]);
        //double enemy_current_speed = history[history.size()-1].second * .95 + getAverageSpeed(history) *.05;
        //double enemy_current_speed = history[history.size()-1].second + getAverageAcceleration(history);

        if(std::abs(my_vec.twoDimensionalCrossProduct(enemy_vec)) == .0) { //parallel
            EV << "  cancel: we are parallel" << name << std::endl;
            continue;
        }

        { // calculate intersection, see https://stackoverflow.com/a/565282
            double my_n = (enemy_current_pos - my_current_pos).twoDimensionalCrossProduct(enemy_vec) / my_vec.twoDimensionalCrossProduct(enemy_vec);
            double enemy_n = (my_current_pos - enemy_current_pos).twoDimensionalCrossProduct(my_vec) / enemy_vec.twoDimensionalCrossProduct(my_vec);

            if(my_n < 0 || enemy_n < 0) // seems we'd have to drive backwards to reach that point...
                continue;

            meeting_point = my_current_pos + (my_vec * my_n);

        }

        { // calculate the meeting time
            // cant use true, maybe because its not perfectly "on their route"? Todo: fix this..
            //double dist_my = traci->getDistance(my_current_pos, meeting_point, false);
            //double dist_enemy = traci->getDistance(enemy_current_pos, meeting_point, false);

            double dist_my = (meeting_point - my_current_pos).length();
            double dist_enemy = (meeting_point - enemy_current_pos).length();

            //double enemy_time = dist_enemy / (enemy_current_speed + (getAverageAcceleration(history) / dist_enemy) );
            //double my_time = dist_my / (my_current_speed + (getAverageAcceleration(myDrivingHistory) / dist_my) );

            // siehe https://johannes-strommer.com/formeln/weg-geschwindigkeit-beschleunigung-zeit/#cc-m-11387728321
            double my_acc = getAverageAcceleration(myDrivingHistory);
            double my_time = -my_current_speed/my_acc + sqrt((pow(my_current_speed, 2)/pow(my_acc,2)) + ((2*dist_my)/my_acc));

            double enemy_acc = getAverageAcceleration(history);
            double enemy_time = -enemy_current_speed/enemy_acc + sqrt((pow(enemy_current_speed, 2)/pow(enemy_acc,2)) + ((2*dist_enemy)/enemy_acc));

            double time_diff = std::abs(enemy_time - my_time);

            if(time_diff < criticalMeetingDuration) {
                //double middle = (sqrt(pow(enemy_time, 2.0)) + sqrt(pow(my_time,2.0)))/2.0;

                EV << "  planning meeting with " << name << " in " << my_time << std::endl;
                meetings[enemy_name] = simTime() + my_time; // + middle;
            } else {
                EV << "  " << "cancel, criticalMeetingDuration not reached:" << time_diff << std::endl;
            }
        }
    }

    announceNextMeeting();
}

void InteractingVehicle::onInterVehicleMessage(InterVehicleMessage *ivmsg) {
    EV << "[" << getParentModule()->getFullName() << "]"
            << "Got a new InterVehicleMessage from " << ivmsg->getName() << "with position " << ivmsg->getPosition() << ", Speed: " << ivmsg->getSpeed() << std::endl;

    enemysDrivingHistory[ivmsg->getName()].push_back({ ivmsg->getPosition(), ivmsg->getSpeed(), simTime() });

    calculateMeetings();
}

void InteractingVehicle::handlePositionUpdate(cObject* obj) {
    DemoBaseApplLayer::handlePositionUpdate(obj);

    EV << "[" << getParentModule()->getFullName() << "]" << "handling position update event "
            << "with position " << mobility->getPositionAt(simTime()) << ", Speed: " << mobility->getSpeed() << std::endl;

    int size = myDrivingHistory.size();
    auto pos = mobility->getPositionAt(simTime());
    if(size > 0 && std::get<0>(myDrivingHistory[size-1]) == pos) {
        // do nothing
        EV << "position not update, not pushing back" << std::endl;
    } else {
        myDrivingHistory.push_back({ pos, mobility->getSpeed(), simTime() });
        calculateMeetings();
    }
}

std::pair<std::string, simtime_t> InteractingVehicle::getNextMeetingTime(int offset) {
    std::pair<std::string, simtime_t> earliest_meeting = { "", simtime_t::getMaxTime() };

    if(offset >= meetings.size())
        return earliest_meeting;

    std::map<std::string, simtime_t> meetings_copy = meetings;

    while(offset >= 0) {
        offset--;

        for(const auto& [name, time] : meetings_copy)
            if (time < earliest_meeting.second && time >= simTime())
                earliest_meeting = { name, time };

        if(offset > 0) { // TODO: nicht die beste lösung...
            meetings_copy.erase(earliest_meeting.first);
            earliest_meeting = { "", simtime_t::getMaxTime() };
        }
    }

    return earliest_meeting;
}

// returns true if from left, in all other cases (even in failures) returns false
bool InteractingVehicle::isFromLeft(std::string name) {
    auto enemys_hist = enemysDrivingHistory[name];
    if(enemys_hist.size() < 2 ||
       myDrivingHistory.size() < 2)
        return false;

    veins::Coord my_current_pos = std::get<0>(myDrivingHistory[myDrivingHistory.size()-1]);
    veins::Coord my_last_pos = std::get<0>(myDrivingHistory[myDrivingHistory.size()-2]);
    veins::Coord enemys_current_pos = std::get<0>(enemys_hist[enemys_hist.size()-1]);

    double d = ((enemys_current_pos - my_last_pos).twoDimensionalCrossProduct(my_current_pos - my_last_pos))/(my_current_pos - my_last_pos).length();
    return d < 0;
}

// get the first meeting where we would be the car from the left
std::pair<std::string, simtime_t> InteractingVehicle::getNextMeetingFromLeft() {
    auto next_meeting = getNextMeetingTime();
    int off = 0;

    while(!isFromLeft(next_meeting.first) &&          // not from left
          next_meeting.second != simtime_t::getMaxTime()) {  // nothing more to do, all iterated

        next_meeting = getNextMeetingTime(off);
        off++;
    }

    return next_meeting;
}

void InteractingVehicle::announceNextMeeting() {
    auto next_meeting = getNextMeetingTime();
    simtime_t time = next_meeting.second;

    if(sendMTWEvt->isScheduled())
        cancelEvent(sendMTWEvt);
    if(sendMCWEvt->isScheduled())
        cancelEvent(sendMCWEvt);
    if(sendMBEvt->isScheduled())
        cancelEvent(sendMBEvt);

    if(time == simtime_t::getMaxTime())
        return;

    simtime_t schedulingTimeWarnTime = time - meetTimeWarnBefore;
    simtime_t schedulingCollisionWarnTime = time - meetCollissionWarnBefore;

    if(simTime() >= schedulingTimeWarnTime && simTime() <= time) // should have warned earlier
        scheduleAt(simTime(), sendMTWEvt);
    else if(simTime() < schedulingTimeWarnTime) // warn correctly
        scheduleAt(schedulingTimeWarnTime, sendMTWEvt);

    if(simTime() >= schedulingCollisionWarnTime && simTime() <= time) // should have warned earlier
        scheduleAt(simTime(), sendMCWEvt);
    else if(simTime() < schedulingCollisionWarnTime) // warn correctly
        scheduleAt(schedulingCollisionWarnTime, sendMCWEvt);

    next_meeting = getNextMeetingFromLeft();
    time = next_meeting.second;

    if(time == simtime_t::getMaxTime())
        return; // no meeting found

    simtime_t schedulingBreakTime = time - meetBreakBefore;

    if(simTime() > schedulingBreakTime && simTime() <= time)
        schedulingBreakTime = simTime();

    if(simTime() <= schedulingBreakTime) {
        if(sendDriveAgainEvt->isScheduled())
            cancelEvent(sendDriveAgainEvt);
        scheduleAt(schedulingBreakTime + breakDuration, sendDriveAgainEvt);

        scheduleAt(schedulingBreakTime, sendMBEvt);
    }
}

void InteractingVehicle::finish()
{
    DemoBaseApplLayer::finish();
    // maybe you want to record some scalars?
}

void InteractingVehicle::continueDriving()
{
    findHost()->bubble("Driving");
    getParentModule()->getDisplayString().setTagArg("i", 1, "green");
    traciVehicle->setSpeed(50);
}

void InteractingVehicle::handleSelfMsg(cMessage* msg)
{
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
    DemoBaseApplLayer::handleSelfMsg(msg);

    switch (msg->getKind()) {
        case SEND_PS_EVT: {

            veins::Coord my_current_pos = std::get<0>(myDrivingHistory[myDrivingHistory.size()-1]);
            double my_current_speed = std::get<1>(myDrivingHistory[myDrivingHistory.size()-1]);

            if(last_sent == my_current_pos)
                EV << "Position already sent. Not resending packet" << std::endl;
            else {
                last_sent = my_current_pos;
                auto wiv = new InterVehicleMessage(getParentModule()->getFullName());

                wiv->setPosition(my_current_pos);
                wiv->setSpeed(my_current_speed);
                wiv->setChannelNumber(static_cast<int>(veins::Channel::cch));

                sendDown(wiv);
            }

            scheduleAt(simTime() + psInterval, sendPSEvt);
            break;
        }
        case SEND_MTW_EVT: { // TimeWarning triggered
            auto next_meeting = getNextMeetingTime();

            if(next_meeting.second == simtime_t::getMaxTime() ||  // meeting was deleted
                    timeWarnHistory[next_meeting.first]) // we warned before
                break;

            timeWarnHistory[next_meeting.first] = true;

            EV << getParentModule()->getFullName() << "warning, " << next_meeting.first << "is near!" << std::endl;
            findHost()->bubble("Timewarning");
            getParentModule()->getDisplayString().setTagArg("i", 1, "orange");
            break;
        }
        case SEND_MTC_EVT: { // CollisionWarning triggered
            auto next_meeting = getNextMeetingTime();

            if(next_meeting.second == simtime_t::getMaxTime() || // meeting was deleted
                    colissionWarnHistory[next_meeting.first]) // we warned before
                break;

            colissionWarnHistory[next_meeting.first] = true;

            EV << getParentModule()->getFullName() << "warning, " << next_meeting.first << "is near!" << std::endl;
            findHost()->bubble("Collisionwarning");
            getParentModule()->getDisplayString().setTagArg("i", 1, "red");
            break;
        }
        case SEND_MB_EVT: { // Breaking triggered
            auto next_meeting = getNextMeetingFromLeft();
            if(next_meeting.second == simtime_t::getMaxTime())
                break; // no meeting found

            if(!breakWarnHistory[next_meeting.first]) { // we did not announce the breaking
                EV << getParentModule()->getFullName() << ": Breaking in favor of enemy car " << next_meeting.first << std::endl;
                findHost()->bubble("Breaking!");
            }

            breakWarnHistory[next_meeting.first] = true;

            traciVehicle->setSpeed(0);
            getParentModule()->getDisplayString().setTagArg("i", 1, "grey");

            break;
        }
        case SEND_DRIVE_AGAIN_EVT: {
            auto next_meeting = getNextMeetingFromLeft();
            simtime_t time = next_meeting.second;

            // delete all histories, worst we can get are false negatives
            breakWarnHistory.empty();
            timeWarnHistory.empty();
            colissionWarnHistory.empty();

            auto schedulingBreakTime = time - meetBreakBefore;
            if(time < simtime_t::getMaxTime() || (simTime() <= time && simTime() >= schedulingBreakTime)) { // do not drive
                scheduleAt(simTime() + breakDuration, sendDriveAgainEvt);
            } else { // drive
                continueDriving();
            }
            break;
        }
    }

}
