#include "Catcher.h"
#include "World.h"

Point2D Catcher::Move(CatWorld* world) {
  //store base vars
  auto side = world->getWorldSideSize() / 2;
  auto cat = world->getCat();
  
  //create frontier
  DjikstraQueue frontier = DjikstraQueue();

  //setup initial conditions
  frontier.push(WeightCell(0.0f, cat));
  CameFrom cameFrom = CameFrom();
  CostSoFar costSoFar = CostSoFar();
  cameFrom.emplace(cat, cat);
  costSoFar.emplace(cat, 0.0f);
  
  // create interator variables for evaluation
  unsigned int debugEscape = 0;
  Point2D current, dest = cat;
  float newCost;

  // until no more options to evaluate
  while (!frontier.empty() && debugEscape < 1000) {
    // store current
    current = frontier.top().mPos;
    frontier.pop();

    // check if at goal
    if (std::abs(current.x) == side || std::abs(current.y) == side) {
      dest = current;
      break;
    }

    // for each valid neighbor
    for (Point2D next : world->neighbors(current)) {
      // checks within bounds
      if (!world->isValidPosition(next)) continue;

      // skip if invalid
      if (world->getContent(next)) continue;

      // calculate new cost
      newCost = costSoFar[current] + 1.0f; //+ edgeHeuristic(current, world);

      // first case, if cell has never been visited, add it to cost list
      if (!costSoFar.contains(next)) {
        frontier.push(WeightCell(newCost, next));
        costSoFar.emplace(next, newCost);
        cameFrom.emplace(next, current);
      }
      // next case, see if new cost is better
      else if (newCost < costSoFar[next]) {
        frontier.push(WeightCell(newCost, next));
        costSoFar[next] = newCost;
        cameFrom[next] = current;
      }
    }

    // debug handling to guard against infinite loops
    debugEscape++;
    if (debugEscape >= 1000) std::cout << "debug escape" << std::endl;
  }

  // compose the path
  Point2D iterator;
  std::vector<Point2D> path = std::vector<Point2D>();

  // backwards trace to generate the path
  return dest;
}
