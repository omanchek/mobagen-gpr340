#ifndef CAT_H
#define CAT_H

#include "Agent.h"

class Cat : public Agent {
public:
  explicit Cat() : Agent(){};
  Point2D Move(CatWorld*) override;
  float heuristic(const Point2D& pos, CatWorld* world) override;
};

#endif  // CAT_H
