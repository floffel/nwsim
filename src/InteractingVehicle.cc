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

        veins::Coord enemy_last_pos = enemys_last_position[ivmsg->getName()];
        veins::Coord enemy_current_pos = ivmsg->getPosition();
        veins::Coord enemy_vec = enemy_current_pos - enemy_last_pos;
        double enemy_vec_length = enemy_vec.length() == 0 ? 1 : enemy_vec.length(); // todo: can be deleted, we test on speed == 0
        veins::Coord enemy_norm_vec = enemy_vec/enemy_vec_length;
        double enemy_current_speed = ivmsg->getSpeed();

        veins::Coord my_current_pos = mobility->getPositionAt(simTime());
        veins::Coord my_vec = my_current_pos - my_last_position;
        double my_vec_length = my_vec.length() == 0 ? 1 : my_vec.length(); // todo: can be deleted, we test on speed == 0
        veins::Coord my_norm_vec = my_vec/my_vec_length;
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

        //EV << "heading was: " << enemy_heading.getRad() << " vs " << my_heading.getRad() << " abs: " << abs(enemy_heading.getRad() - my_heading.getRad()) << std::endl;

        //EV << "okay, going to intersect at " << intersection.x << " " << intersection.y << " " << intersection.z << "" << std::endl;


        // works, but is subotimal, will give more coordinates than junctions
        //{ // find my_next_point
        /*    auto* lane = new veins::TraCICommandInterface::Lane(traci, traciVehicle->getLaneId());

            EV << "starting to search for next point " << std::endl;
            for(auto line_point : lane->getShape()) {
                double n_x = (line_point.x - my_current_pos.x) / my_vec.x;
                double n_y = (line_point.y - my_current_pos.y) / my_vec.y;
                double n_diff = (n_x - n_y);

                if( (0 < n_diff) && (n_diff < 4)) {
                    EV << "found nextpoint! " << line_point << std::endl;
                    my_next_point = line_point;
                }
            }
        //}
         *
         */

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

        /*
        // find my_next_point out of all junctions
            EV << "starting to search for next point " << std::endl;

            for(std::string junction_id : traci->getJunctionIds()) {
                veins::TraCICommandInterface::Junction junction = veins::TraCICommandInterface::Junction(traci, junction_id);
                veins::Coord junction_pos = junction.getPosition();

                EV << "checking " << junction_pos << std::endl;

                double n_x = (junction_pos.x - my_current_pos.x) / my_vec.x;
                double n_y = (junction_pos.y - my_current_pos.y) / my_vec.y;
                double n_diff = n_x - n_y;

                EV << "nx: " << n_x << std::endl;
                EV << "ny: " << n_y << std::endl;
                EV << "n_diff: " << n_diff << std::endl;

                if(0 < n_diff && n_diff < 4) {
                    EV << "found nextpoint! " << junction_pos << std::endl;
                    my_next_point = junction_pos;
                    break;
                }
            }
        */


        { // calculate the meeting time
            /*double enemy_n_x = (line_point.x - enemy_current_pos.x) / enemy_vec.x;
            double enemy_n_y = (line_point.y - enemy_current_pos.y) / enemy_vec.y;
            double enemy_n = (n_x + n_y) / 2;

            double my_n_x = (line_point.x - my_current_pos.x) / my_vec.x;
            double my_n_y = (line_point.y - my_current_pos.y) / my_vec.y;
            double my_n = (n_x + n_y) / 2;*/

            // cant use true (for "real" distance, dunno why)
            double dist_my = traci->getDistance(my_current_pos, my_next_point, false);
            double dist_enemy = traci->getDistance(enemy_current_pos, my_next_point, false);

            EV << "my distance: " << dist_my << std::endl;
            EV << "enemy distance: " << dist_enemy << std::endl;


            double enemy_time = dist_enemy / enemy_current_speed;
            double my_time = dist_my / my_current_speed;

            //simtime_t enemy_time = enemy_current_pos.distance(my_next_point) * enemy_current_speed;
            //simtime_t my_time = my_current_pos.distance(my_next_point) * my_current_speed;
            double time_diff = enemy_time - my_time;

            EV << "I would attend meeting @: " << my_time << std::endl;
            EV << "enemy would attend meeting @: " << enemy_time << std::endl;

            EV << "time diff would be: " << time_diff << std::endl;

            if(std::abs(time_diff) < 5) {
                EV << "planning meeting @" << my_time << std::endl;
            }

            //simtime_t my_time = line_point.distance(mobility->getPositionAt(simTime())) * mobility->getSpeed();

        }

        /*

        veins::Coord norm_vec_diff = my_norm_vec - enemy_norm_vec; // TODO: hier müssen wir den größeren - kleineren nehemn


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
        */


    }

}

// vp=vector start point
// v=vector
// ip=intersection point
double InteractingVehicle::calculate_intersection_multiplicator(veins::Coord vp, veins::Coord v, veins::Coord ip) {
    // vp + n * v = ip
    // => n = (ip - vp) / v
    veins::Coord t = ip-vp;
    double n_x = t.x / v.x;
    double n_y = t.y / v.y;
    double n_z = t.y / v.z;

    if(!(n_x == n_y == n_z)) {
        EV << "nx=" << n_x << ", ny=" << n_y << ", nz=" << n_z << std::endl;
        return INFINITY;
    }

    return n_x;
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
