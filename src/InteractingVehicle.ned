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

package veins_project;

import org.car2x.veins.base.modules.IBaseApplLayer;
import org.car2x.veins.modules.application.ieee80211p.DemoBaseApplLayer;

simple InteractingVehicle extends DemoBaseApplLayer like IBaseApplLayer {   
    parameters:
        @class(InteractingVehicle);
        double psInterval = default(2s) @unit(s); // the intervall between 2 ps messages
        double meetWarnBefore = default(7s) @unit(s); // the duration a car has before to meet when a warning shall appear
        double meetBreakBefore = default(3s) @unit(s); // the duration a car is before to meet when a break shall be initiated
		double criticalMeetingDuration = default(1s) @unit(s); // todo: a radius would be nicer
		double breakDuration = default(3s) @unit(s); // the duration before a car shall resume after a meeting
}
