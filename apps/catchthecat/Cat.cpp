#include "Cat.h"
#include <queue>
#include <unordered_map>
#include <utility>
#include "World.h"
#include <stdexcept>

//define the queue to use
struct WeightCell
{
  WeightCell(float weight, Point2D pos) { mWeight = weight; mPos = pos; }

  bool operator<(const WeightCell& rhs) { return mWeight < rhs.mWeight; }
  bool operator>(const WeightCell& rhs) { return mWeight > rhs.mWeight; }

  float mWeight;
  Point2D mPos;
};

bool operator>(const WeightCell& lhs, const WeightCell& rhs) { return lhs.mWeight > rhs.mWeight; }

using DjikstraQueue = std::priority_queue<WeightCell, std::vector<WeightCell>, std::greater<WeightCell>>;
using CameFrom = std::unordered_map<Point2D, Point2D>;
using CostSoFar = std::unordered_map<Point2D, float>;

float edgeHeuristic(const Point2D& pos, CatWorld* world)
{
  //declare an array to get distances to edges
  float distances[4]; //up, down, left, right
  int size = world->getWorldSideSize();
  
  //get distances to each edge
  distances[0] = std::abs((-1 * size) - pos.y);
  distances[1] = std::abs((size) - pos.y);
  distances[2] = std::abs((-1 * size) - pos.x);
  distances[3] = std::abs((size)-pos.x);

  //pick smallest edge distance
  int min = 0;
  for (int i = 1; i < 4; i++)
  {
    if (distances[i] < distances[min]) min = i;
  }

  //determine valid neighbors
  int blockedNeighbors = 0;
  for (Point2D it : world->neighbors(pos))
  {
    //don't count past edge
    if (!world->isValidPosition(it)) continue;

    if (world->getContent(it)) blockedNeighbors++;
  }

  //weight the cell based on distance to edge and how open it is
  return distances[min] + blockedNeighbors;
}


Point2D Cat::Move(CatWorld* world) {
  auto rand = Random::Range(0, 5);
  auto pos = world->getCat();

  std::cout << pos.x << pos.y << std::endl;
  
  //initial setup
  DjikstraQueue frontier = DjikstraQueue();
  
  frontier.push(WeightCell(0.0f, pos));
  CameFrom cameFrom = CameFrom();
  CostSoFar costSoFar = CostSoFar();
  cameFrom.emplace(pos, pos);
  costSoFar.emplace(pos, 0.0f);

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
        newCost = costSoFar[current] + 1.0f + edgeHeuristic(current, world);

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

      debugEscape++;

      if (debugEscape >= 1000) std::cout << "debug escape" << std::endl;
  }

  //compose the path
  Point2D iterator;
  std::vector<Point2D> path = std::vector<Point2D>();

  //backwards trace
  iterator = dest;
  while (iterator != pos)
  {
    path.push_back(iterator);
    iterator = cameFrom[iterator];
  }

  if (path.size() <= 0) return pos;

  for (int i = 0; i < path.size(); i++) {
    std::cout << path[i].x << ", " << path[i].y << std::endl;
  }

  return path[path.size() - 1];
}
