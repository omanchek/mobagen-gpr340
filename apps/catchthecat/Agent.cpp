#include "Agent.h"
#include <climits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "World.h"

using namespace std;

std::vector<Point2D> Agent::generatePath(CatWorld* w) {
  unordered_map<Point2D, Point2D> cameFrom;  // to build the flowfield and build the path
  unordered_map<Point2D, float> costSoFar;
  priority_queue<WeightCell, vector<WeightCell>, std::greater<WeightCell>> frontier;                   // to store next ones to visit
  unordered_set<Point2D> frontierSet;        // OPTIMIZATION to check faster if a point is in the queue
  //unordered_map<Point2D, bool> visited;      // use .at() to get data, if the element dont exist [] will give you wrong results
  unordered_set<Point2D> visitedSet;

  //define path
  vector<Point2D> path = vector<Point2D>();

  // bootstrap state
  int side = w->getWorldSideSize() / 2;
  auto catPos = w->getCat();
  frontier.push(WeightCell(0 + heuristic(catPos, w), catPos));
  frontierSet.insert(catPos);
  cameFrom.emplace(catPos, catPos);
  costSoFar.emplace(catPos, 0.0f);
  Point2D borderExit = {INT32_MAX, INT32_MAX};  // sentinel: no border found yet

  Point2D current;
  while (!frontier.empty()) {
    // get the current from frontier
    current = frontier.top().mPos;
    frontier.pop();

    // remove the current from frontierset
    frontierSet.erase(current);

    // mark current as visited
    visitedSet.emplace(current);
    
    // getVisitableNeightbors(world, current) returns a vector of neighbors that are not visited, not cat, not block, not in the queue
    // iterate over the neighs:
    for (Point2D next : w->neighbors(current))
    {
      if (!w->isValidPosition(next)) continue; //invalid check
      if (frontierSet.contains(next)) continue; //skip if already in queue
      if (next == catPos) continue; //ensure not cat
      if (w->getContent(next)) continue; //blocked check
      if (visitedSet.contains(next)) continue; //don't use visited cells

      // for every neighbor set the cameFrom
      // enqueue the neighbors to frontier and frontierset
      cameFrom.emplace(next, current);
      costSoFar.emplace(next, costSoFar[current] + 1);
      frontier.push(WeightCell(costSoFar[next] + 1.0f + heuristic(next, w), next));
      frontierSet.emplace(next);

      // do this up to find a visitable border and break the loop
      if (std::abs(next.x) >= side || std::abs(next.y) >= side)
      {
        path.push_back(next);
        break;
      }

      //otherwise, if there are no valid paths to the end
      else if (frontier.empty())
      {
        //check if can get a random neighbor to use
        Point2D failPos = catPos;
        if (getRandomEmptyNeighbor(catPos, w, failPos))
        {
          path.push_back(failPos);
        }

        return path;
      }
    }        
  }
  
  // if the border is not infinity, build the path from border to the cat using the camefrom map
  // if there isnt a reachable border, just return empty vector
  // if your vector is filled from the border to the cat, the first element is the catcher move, and the last element is the cat move
  if (path.size() <= 0) return path;

  Point2D iterator = path.at(0), source;
  while (iterator != catPos)
  {
    //get the source tile
    source = cameFrom.at(iterator);
    
    //only at it to the path if not the current space
    if (source != catPos) path.push_back(source);

    //update iterator
    iterator = source;
  }
  
  return path;
}

unsigned int getDistanceToEdge(const WorldEdges edge, const Point2D& pos, CatWorld* world)
{
  int side = world->getWorldSideSize() / 2;

  //pick based on edge
  switch (edge)
  {
    case WorldEdges::UP:
      return std::abs(pos.y - (-1 * side));
    case WorldEdges::DOWN:
      return std::abs(pos.y - side);
    case WorldEdges::LEFT:
      return std::abs(pos.x - (-1 * side));
    case WorldEdges::RIGHT:
      return std::abs(pos.x - side);
    default:
      return 0;
  }
}

int getMinDistanceToEdge(const Point2D& pos, CatWorld* world)
{
  // create an array of the edges
  const unsigned int EDGE_COUNT = 4;
  int size = world->getWorldSideSize() / 2;
  int edges[EDGE_COUNT];  // up, down, left, right
  
  //get distances to each edge
  edges[(int)WorldEdges::UP] = std::abs((-1 * size) - pos.y);
  edges[(int)WorldEdges::DOWN] = std::abs((size)-pos.y);
  edges[(int)WorldEdges::LEFT] = std::abs((-1 * size) - pos.x);
  edges[(int)WorldEdges::RIGHT] = std::abs((size)-pos.x);

  //return the min
  return std::min(
                  std::min(edges[(int)WorldEdges::UP], edges[(int)WorldEdges::DOWN]),
                  std::min(edges[(int)WorldEdges::UP], edges[(int)WorldEdges::DOWN])
                 );
}

unsigned int getNumBlockedNeighbors(const Point2D& pos, CatWorld* world)
{
  //define counter
  unsigned int blocked = 0;

  //iterate over neighbors
  for (Point2D it : world->neighbors(pos))
  {
    //don't count invalids
    if (!world->isValidPosition(it)) continue;

    //increment if filled
    if (world->getContent(it)) blocked++;
  }

  return blocked;
}

bool getRandomEmptyNeighbor(const Point2D& origin, CatWorld* world, Point2D& out)
{
  //setup tracking
  Point2D neighbors[6];
  unsigned int numValidNeighbors = 0;

  //check if each neighbor is valid to pick from
  for (Point2D it : world->neighbors(origin))
  {
    //skip if out of bounds or filled
    if (!world->isValidPosition(it)) continue;
    if (world->getContent(it)) continue;

    //otherwise, add to list and increment
    neighbors[numValidNeighbors] = it;
    numValidNeighbors++;
  }

  //if no neighbors were valid, fail
  if (numValidNeighbors <= 0) return false;

  //otherwise, pick a random one to return
  unsigned int index = Random::Range(0, numValidNeighbors - 1);
  out = neighbors[index];
  return true;
}
