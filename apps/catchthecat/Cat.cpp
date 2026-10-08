#include "Cat.h"
#include <utility>
#include "World.h"
#include <stdexcept>

float edgeHeuristic(const Point2D& pos, CatWorld* world)
{
  //sum blocked neighbors of neighbors
  unsigned int secondDegreeBlocked = 0;
  for (Point2D it : world->neighbors(pos))
  {
    //skip out of bounds
    if (!world->isValidPosition(pos)) continue;

    //update count
    secondDegreeBlocked += getNumBlockedNeighbors(it, world);
  }

  secondDegreeBlocked *= ((world->getWorldSideSize() / 2) - getMinDistanceToEdge(pos, world));

  int componentDiff = std::max(std::abs(pos.x - world->getCat().x), std::abs(pos.y - world->getCat().y));

  //weight the cell based on distance to edge and how open it is
  return getMinDistanceToEdge(pos, world) + getNumBlockedNeighbors(pos, world);//+ (world->getWorldSideSize() - componentDiff);
}

float Cat::heuristic(const Point2D& pos, CatWorld* world)
{
  return edgeHeuristic(pos, world);
}

Point2D Cat::Move(CatWorld* world)
{    
  //store current cat position
  auto pos = world->getCat();
  /*
  //initial setup
  DjikstraQueue frontier = DjikstraQueue();
  
  //setup initial state for pathfinding
  frontier.push(WeightCell(0.0f, pos));
  CameFrom cameFrom = CameFrom();
  CostSoFar costSoFar = CostSoFar();
  cameFrom.emplace(pos, pos);
  costSoFar.emplace(pos, 0.0f);

  //create interator variables for evaluation
  Point2D current, dest = pos;
  float newCost;
  int halfSize = world->getWorldSideSize() / 2;

  //until no more options to evaluate
  while (!frontier.empty())
  {
      //clear invalid options
      while (frontier.size() > 0 && world->getContent(frontier.top().mPos)) frontier.pop();
      if (frontier.empty()) break;

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
  }

  //compose the path
  Point2D iterator;
  std::vector<Point2D> path = std::vector<Point2D>();
  */
  std::vector<Point2D> path = generatePath(world);

  //if at goal, return the goal as point to move to
  Point2D move;
  if (path.size() <= 0)
  {
    move = pos;
  }
  else
  {
    move = path.at(path.size() - 1);
  }
  world->lastMove = move;
  return move;
}
