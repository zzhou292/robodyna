// Robodyna public API. These aliases preserve the inherited implementation.
#ifndef ROBODYNA_MBD_RBBODYEASY_H
#define ROBODYNA_MBD_RBBODYEASY_H

#include "chrono/physics/ChBodyEasy.h"

namespace robodyna::mbd {
using RbBodyEasyBox = ::chrono::ChBodyEasyBox;
using RbBodyEasySphere = ::chrono::ChBodyEasySphere;
}  // namespace robodyna::mbd

#endif
