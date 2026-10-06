#include "Cat.h"
#include <utility>
#include "World.h"
#include <stdexcept>

float edgeHeuristic(const Point2D& pos, CatWorld* world)
{
  //determine valid neighbors
  int blockedNeighbors = 0;
  for (Point2D it : world->neighbors(pos))
  {
    //don't count past edge
    if (!world->isValidPosition(it)) continue;

    if (world->getContent(it)) blockedNeighbors++;
  }

  //weight the cell based on distance to edge and how open it is
  return getMinDistanceToEdge(pos, world) + blockedNeighbors;
}


Point2D Cat::Move(CatWorld* world)
{
  //store current cat position
  auto pos = world->getCat();
    
  //initial setup
  DjikstraQueue frontier = DjikstraQueue();
  
  //setup initial state for pathfinding
  frontier.push(WeightCell(0.0f, pos));
  CameFrom cameFrom = CameFrom();
  CostSoFar costSoFar = CostSoFar();
  cameFrom.emplace(pos, pos);
  costSoFar.emplace(pos, 0.0f);

  //create interator variables for evaluation
  unsigned int debugEscape = 0;
  Point2D current, dest = pos;
  float newCost;
  int halfSize = world->getWorldSideSize() / 2;

  //until no more options to evaluate
  while (!frontier.empty() && debugEscape < 1000)
  {
      //store current
      current = frontier.top().mPos;
      frontier.pop();

      //check if at goal
      if (std::abs(current.x) == halfSize || std::abs(current.y) == halfSize)
      {
        dest = current;
        break;
      }

      //for each valid neighbor
      for (Point2D next : world->neighbors(current))
      {
        //checks within bounds
        if (!world->isValidPosition(next)) continue;

        //skip if invalid
        if (world->getContent(next)) continue;

        //calculate new cost
        newCost = costSoFar[current] + 1.0f + edgeHeuristic(next, world);

        //first case, if cell has never been visited, add it to cost list
        if (!costSoFar.contains(next)) {
          frontier.push(WeightCell(newCost, next));
          costSoFar.emplace(next, newCost);
          cameFrom.emplace(next, current);
        }
        //next case, see if new cost is better
        else if (newCost < costSoFar[next]) {
          frontier.push(WeightCell(newCost, next));
          costSoFar[next] = newCost;
          cameFrom[next] = current;
        }
      }

      //debug handling to guard against infinite loops
      debugEscape++;
      if (debugEscape >= 1000) std::cout << "debug escape" << std::endl;
  }

  //compose the path
  Point2D iterator;
  std::vector<Point2D> path = std::vector<Point2D>();

  //backwards trace to generate the path
  iterator = dest;
  while (iterator != pos)
  {
    path.push_back(iterator);
    iterator = cameFrom[iterator];
  }

  //if at goal, return the goal as point to move to
  if (path.size() <= 0) return pos;

  //otherwise, grab the next point on the path
  return path[path.size() - 1];
}
