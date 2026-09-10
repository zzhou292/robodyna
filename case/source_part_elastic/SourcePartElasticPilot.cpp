#include "SourcePartElasticPilot.h"
#include <stdexcept>

namespace crash::cases::source_part_elastic {
Config PilotConfig(unsigned refinement) {
    if(refinement!=1 && refinement!=2 && refinement!=4)
        throw std::invalid_argument("Source-part refinement must be 1, 2 or 4");
    Config c;
    c.dt=PilotStep/refinement;
    c.pulse_duration=4096*PilotStep;
    c.acceleration=10000;
    c.spatial_axis=2;
    c.direction={1,0,0};
    c.maximum_displacement=.02;
    c.maximum_rotation=.25;
    c.maximum_strain=.005;
    c.maximum_thickness_curvature=.01;
    c.minimum_area_ratio=.99;
    c.maximum_area_ratio=1.01;
    c.minimum_thickness_ratio=.99;
    c.maximum_thickness_ratio=1.01;
    c.maximum_energy_residual=1e-10;
    c.relative_energy_residual=.05;
    c.maximum_native_dt_fraction=.125;
    c.configuration_id=0x53504550494c3031ULL;
    c.qualification_id=0x535045454c413031ULL;
    return c;
}
}
