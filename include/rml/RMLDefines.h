#ifndef RML_DEFINES_H
#define RML_DEFINES_H

#include <string>

namespace rml {


enum JacobianObserver {VehicleFrame, InertialFrame};

namespace FrameID{

     const std::string Tool = "_Tool";
     const std::string Joint = "_Joint_";
     const std::string Body = "_Body_";
     const std::string Manipulability = "_Manipulability_";
}
}


#endif // RML_DEFINES_H
