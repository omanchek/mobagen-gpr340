#include "CohesionRule.h"
#include <glm/glm.hpp>

glm::vec2 CohesionRule::computeForce(const std::vector<BoidView>& neighborhood, const BoidView& boid) {
  glm::vec2 cohesionForce(0.f);

  // glm::length(vec) returns the length of a vector,
  // glm::normalize(vec) returns the normalized vector (length 1) in the same direction as vec.

  // begin solution

  //edge case for no neighbors
  if (neighborhood.size() <= 0) return glm::vec2();

  //calculate the center of mass of the neighborhood
  glm::vec2 centerOfMass = glm::vec2();
  for (int i = 0; i < neighborhood.size(); i++)
  {
    centerOfMass += neighborhood[i].position;
  }
  centerOfMass = centerOfMass / static_cast<float>(neighborhood.size());

  //get the cohesion force to use
  cohesionForce = (centerOfMass - boid.position);

  return glm::normalize(cohesionForce);
}
