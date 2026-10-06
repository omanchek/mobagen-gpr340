#include "Catcher.h"
#include "World.h"

WorldEdges getGoalEdge(const Point2D& goal, CatWorld* world)
{ 
  //get half size
  auto side = world->getWorldSideSize() / 2;
  
  //determine which edge the given goal is on
  if (goal.x == -1 * side) return WorldEdges::LEFT;
  else if (goal.x == side) return WorldEdges::RIGHT;
  else if (goal.y == -1 * side) return WorldEdges::UP;
  else return WorldEdges::DOWN;
}

float calculateHeuristic(const Point2D& pos, CatWorld* world, float distScalar = 1.0f, float blockedScalar = 2.0f)
{
  //get the cat's position
  auto cat = world->getCat();
  auto side = world->getWorldSideSize() / 2;

  //determine the minimum distance from the cat and the pos to an edge
  int posMin = side - std::max(std::abs(pos.x - cat.x), std::abs(pos.y - cat.y));//getMinDistanceToEdge(pos, world);
  int catMin = getMinDistanceToEdge(cat, world) - 1;

  int distProduct = std::max(0, catMin) + std::max(0, posMin);

  int blockedNeighbors = 1;
  for (Point2D it : world->neighbors(pos))
  {
    if (!world->isValidPosition(it)) continue;

    if (world->getContent(pos)) blockedNeighbors++;
  }

  return ((distProduct * distScalar) + (std::pow(blockedNeighbors, 2) * blockedScalar)) * std::max(0, (catMin));
}

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

      //skip if cat
      if (world->getCat() == next) continue;

      // calculate new cost
      newCost = std::max(0.0f, costSoFar[current] + 1.0f + calculateHeuristic(next, world));
      //std::cout << (std::abs(costSoFar[current] - calculateHeuristic(next, world))) << std::endl;

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

  // backwards trace to generate the path
  std::vector<Point2D> path = std::vector<Point2D>();
  Point2D iterator = dest;
  while (iterator != cat) {
    path.push_back(iterator);
    iterator = cameFrom[iterator];
  }

  return path.at(0);
}
