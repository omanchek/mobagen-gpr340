#include "AlignmentRule.h"
#include <glm/glm.hpp>

glm::vec2 AlignmentRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 averageVelocity(0.f);
  // glm::vec2 can be divided by a float, which will divide each component of the vector by that float.

  // begin solution

  //edge case for no neighbors
  if (neighborhood.size() <= 0) return averageVelocity;

  //average the velocity of neighborhood
  for (int i = 0; i < neighborhood.size(); i++)
  {
    averageVelocity += neighborhood[i].velocity;
  }

  //add in this boid's velocity, and determine average
  averageVelocity = averageVelocity / static_cast<float>(neighborhood.size());

  return averageVelocity;
  // end solution
}
