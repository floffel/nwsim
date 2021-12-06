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

InteractingVehicle::~InteractingVehicle() {
    cancelAndDelete(sendPSEvt);
}

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
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
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

        //std::vector<std::tuple<point3d, double>> history = seen[ivmsg->getName()];
        //std::tuple<point3d, double> t = new std::tuple<point3d, double>(ivmsg->getPosition(), ivmsg->getSpeed());
        //history.push_back(std::make_tuple(ivmsg->getPosition(), ivmsg->getSpeed()));
        //std::tuple<veins::Coord, double> t = {ivmsg->getPosition(), ivmsg->getSpeed()};
        //auto t = std::tuple<veins::Coord, double> {ivmsg->getPosition(), ivmsg->getSpeed()};

        // todo: would be cool to have some type that holds exactly 2 values
        // and overloads << with pushing to the end, effectifly deleting the last vector
        // todo: maybe just use an array of [2] for now
        // as we do have a memory rip in here
        auto history = seen[ivmsg->getName()];
            history.push_back({ivmsg->getPosition(), ivmsg->getSpeed()});
        seen[ivmsg->getName()] = history;

        if(history.size() == 1) // nothing to calculate, if we only saw the car one time
            return;

        // calculate the vector:
        Coord enemy_vec;
        double enemy_speed;
        {
            auto second_last = history[history.size() - 2];
            auto last = history[history.size() - 1];

            enemy_vec = last.first - second_last.first;
            enemy_speed = last.second;
        }

        // calculate meeting points, update if already
        //auto my_speed = traciVehicle->getSpeed();
        //vec3d
        //traciVehicle->getLaneId()
        { // get the directen this car is traveling to
            auto angle = traciVehicle->getAngle();
            double lanepos = traciVehicle->getLanePosition();
            auto* lane = new veins::TraCICommandInterface::Lane(traci, traciVehicle->getLaneId());
            EV << "Line shapes: " << std::endl;

            // wir brauchen ein radius zum checken auf gleichheit
            for(auto line_point : lane->getShape()) {
                // wir bekommen mehrere punkte, wir müssen also herausbekommen, welcher der nächste ist, die restlichen können wir ignorieren
                // das währe in der realen welt vmtl. ein problem, wenn man kurven fahren würde




                EV << "Traveling to x: " << line_point.x << " y:" << line_point.y << " z:" << line_point.z << std::endl;

                veins::Coord my_current_pos = mobility->getPositionAt(simTime());
                veins::Coord my_way_vec = line_point - my_current_pos;


                veins::Coord my_norm_vec = my_way_vec/my_way_vec.length();
                veins::Coord enemy_norm_vec = enemy_vec/enemy_vec.length();

                veins::Coord norm_vec_diff = my_norm_vec - enemy_norm_vec; // TODO: hier müssen wir den größeren - kleineren nehemn
                // check for same vectors, as we drive into the same direction then, todo: use a better threshhold

                EV << "my norm_vec.x: " << my_norm_vec.x << "norm_vec.y: " << my_norm_vec.y << "norm_vec.z: " << my_norm_vec.z << std::endl;
                EV << "enemy norm_vec.x: " << enemy_norm_vec.x << "norm_vec.y: " << enemy_norm_vec.y << "norm_vec.z: " << enemy_norm_vec.z << std::endl;

                EV << "norm_vec_diff.x: " << norm_vec_diff.x << "norm_vec_diff.y: " << norm_vec_diff.y << "norm_vec_diff.z: " << norm_vec_diff.z << std::endl;

                if(norm_vec_diff.x == 0 && norm_vec_diff.y == 0 && norm_vec_diff.z == 0)
                    continue;

                // genormte vektoren subtrahieren und mit 0 vergleichen wäre super
                //if(my_way_vec)

                // check if we the point is inbound of enemy_vec
                // todo: proposal to implement the following behavior :)
                //veins::Coord diff = line_point / enemy_vec; //{ coord.x / enemy_vec.x, coord.y / enemy_vec.y, coord.z / enemy_vec.z };
                veins::Coord* diff = new veins::Coord(line_point.x / enemy_vec.x, line_point.y / enemy_vec.y, line_point.z / enemy_vec.z);

                // check if we are inbound
                if(abs(diff->x - diff->y) > same_point_radius_threshold ||
                   abs(diff->x - diff->z) > same_point_radius_threshold ||
                   abs(diff->y - diff->z) > same_point_radius_threshold)
                    continue;

                // this won't calculate any curves etc.!
                simtime_t enemy_time = enemy_vec.distance(line_point) * enemy_speed;
                simtime_t my_time = line_point.distance(mobility->getPositionAt(simTime())) * mobility->getSpeed();

                if(abs(enemy_time.raw() - my_time.raw()) > same_time_threshold.raw())
                    continue;

                // todo: eigentlich brauchen wir den punkt garnicht
                meeting_points[ivmsg->getName()] = { line_point, my_time };
                EV << "[" << getParentModule()->getFullName() << "]" << "added meeting time with " << ivmsg->getName() << " in " << my_time << ", current time is " << simTime() << std::endl;
            }
        }


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
    //DemoBaseApplLayer::handleSelfMsg(msg); this will schedule a bacon message, which interferes with the InterVehicleMessage

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
    }

}
