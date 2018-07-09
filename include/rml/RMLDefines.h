#ifndef RML_DEFINES_H
#define RML_DEFINES_H

#include <string>

namespace rml {

enum JacobianType { Vehicle, Arm };

namespace FrameID{

     const std::string Tool = "_Tool";
     const std::string Joint = "_Joint_";
     const std::string Body = "_Body_";
}
}


#endif // RML_DEFINES_H
