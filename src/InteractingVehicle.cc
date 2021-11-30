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

void InteractingVehicle::initialize(int stage)
{
    DemoBaseApplLayer::initialize(stage);
    if (stage == 0) {
        // Initializing members and pointers of your application goes here
        sendMsg(generateMsg());
    }
    else if (stage == 1) {
        // Initializing members that require initialized other modules goes here
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
        EV << "Got a new InterVehicleMessage from " << ivmsg->getName() << "with position " << ivmsg->getPosition() << ", Speed: " << ivmsg->getSenderSpeed() << std::endl;
}

void InteractingVehicle::finish()
{
    DemoBaseApplLayer::finish();
    // maybe you want to record some scalars?
}

void InteractingVehicle::handleSelfMsg(cMessage* msg)
{
    EV << "new self message" << std::endl;
    DemoBaseApplLayer::handleSelfMsg(msg);
    // this method is for self messages (mostly timers)
    // it is important to call the DemoBaseApplLayer function for BSM and WSM transmission
}

InterVehicleMessage* InteractingVehicle::generateMsg() {

    InterVehicleMessage* msg = new InterVehicleMessage(getParentModule()->getFullName());

    //const auto mobility = TraCIMobilityAccess().get(getParentModule());

    //auto position = mobility->getPositionAt(simTime()); // DemoBaseApplLayer::curPosition
    //auto speed = mobility->getCurrentSpeed(); // DemoBaseApplLayer::curSpeed

    auto position = DemoBaseApplLayer::curPosition;
    auto speed = DemoBaseApplLayer::curSpeed;


    EV << "adding with current position: " << position << std::endl;
    EV << "adding with current speed: " << speed << std::endl;
    msg->setSenderPos(position);
    msg->setSenderSpeed(speed);
    msg->setChannelNumber(static_cast<int>(veins::Channel::cch));
    return msg;
}

void InteractingVehicle::sendMsg(InterVehicleMessage* msg) {
    EV << "Sending message!";
    //send(msg, "lowerLayerOut");
    //DemoBaseApplLayer::handleLowerMsg(msg);
    sendDown(msg);
}

