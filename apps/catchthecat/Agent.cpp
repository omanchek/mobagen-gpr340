#include "Agent.h"
#include <climits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include "World.h"

using namespace std;

std::vector<Point2D> Agent::generatePath(CatWorld* w) {
  unordered_map<Point2D, Point2D> cameFrom;  // to build the flowfield and build the path
  queue<Point2D> frontier;                   // to store next ones to visit
  unordered_set<Point2D> frontierSet;        // OPTIMIZATION to check faster if a point is in the queue
  unordered_map<Point2D, bool> visited;      // use .at() to get data, if the element dont exist [] will give you wrong results

  // bootstrap state
  auto catPos = w->getCat();
  frontier.push(catPos);
  frontierSet.insert(catPos);
  Point2D borderExit = {INT32_MAX, INT32_MAX};  // sentinel: no border found yet

  while (!frontier.empty()) {
    // get the current from frontier
    // remove the current from frontierset
    // mark current as visited
    // getVisitableNeightbors(world, current) returns a vector of neighbors that are not visited, not cat, not block, not in the queue
    // iterate over the neighs:
    // for every neighbor set the cameFrom
    // enqueue the neighbors to frontier and frontierset
    // do this up to find a visitable border and break the loop
  }

  // if the border is not infinity, build the path from border to the cat using the camefrom map
  // if there isnt a reachable border, just return empty vector
  // if your vector is filled from the border to the cat, the first element is the catcher move, and the last element is the cat move
  return vector<Point2D>();
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

  //find min
  int min = 0;
  for (int i = 0; i < EDGE_COUNT; i++)
  {
    if (edges[i] < edges[min]) min = i;
  }

  return min;
}
