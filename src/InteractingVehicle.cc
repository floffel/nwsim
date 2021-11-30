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

InteractingVehicle::~InteractingVehicle() {
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
