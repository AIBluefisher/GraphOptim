#ifndef APPLICATIONS_UTILS_H_
#define APPLICATIONS_UTILS_H_

#include <string>
#include <vector>

#include <gopt/geometry/track_builder.h>

namespace gopt {

void LoadTracksFromDB(
    const std::string& database_path,
    TrackElements* track_elements,
    std::vector<std::pair<track_t, track_t>>* track_element_pairs);

}

#endif  // APPLICATIONS_UTILS_H_
