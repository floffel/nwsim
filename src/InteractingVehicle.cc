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

            // getRoadId()
            // oder getLaneId
            // getLanePosition
            // getPlannedRoadIds
            // wir brauchen noch eine methode, um zu entscheiden, wann er wo sein wird (Move kann getPositionAt),
            // wir können so eine zeit berechnen, wann er am "anfang" eines vectors und wann am "ende" eines vectors ist, um so zu vergleichen ob es im selben
            //   zeitradius ist
            // dass dann in ein tuple mit dem vector aufnehmen
            //   dann die schneidenden vectors (schneidefunktion gibt es nicht, aber siehe tracicommandinterface.h->getDistance)
            //   und die zeit vergleichen und ein threshhold einbauen
            // so kann dann erkannt werden ob die autos crashen

            // wir brauchen die lane, um die shape zu bekommen

            // calculate meeting points for the street we are on and every street we are goint to approach
            //std::vector<std::string> road_ids = { traciVehicle.getRoadId() };

            std::vector<std::string> possible_lanes;

            possible_lanes.push_back(traciVehicle->getLaneId());

            /*EV << "planning to use roads: ";
            for(std::string road : traciVehicle->getPlannedRoadIds()) {
                EV << road;
            }
            EV << std::endl;*/

            //auto* first = new veins::TraCICommandInterface::Road(traci, traciVehicle->getRoadId());
            EV << "my lanes are: " << std::endl;
            for(std::string l : possible_lanes) {
                //possible_lanes.push_back(l);
                EV << l << " ";

                auto lane = veins::TraCICommandInterface::Lane(traci, l);
                EV << "mean speed: " << lane.getMeanSpeed() << " ";
                //lane.getMeanSpeed();
                //EV << "shape: " << lane.getShape() << " ";

                //lane.getShape();
            }
            EV << std::endl;


            // getLaneIds()
            /*
            {
                auto* road = new veins::TraCICommandInterface::Road(traci, traciVehicle->getRoadId());
                auto* move = new veins::Move();
                //move->setDirectionByTarget();
                move->setSpeed(road->getMeanSpeed());
            }
            */

            //road_ids


            /*for(string t : traciVehicle->getRoadIds()) {

            }*/

            //drive_positions


            // vielleicht mit einem Move arbeiten?
            // oder mit Heading?

            // wie bekommt man die road?
            // man kann road -> lanes -> getShape (Alles in traciCommandInterface)

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

    // put handling of messages from other nodes here

    if(InterVehicleMessage *ivmsg = check_and_cast<InterVehicleMessage *>(msg))
        EV << "Got a new InterVehicleMessage from " << ivmsg->getName() << "with position " << ivmsg->getPosition() << ", Speed: " << ivmsg->getSpeed() << std::endl;
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
