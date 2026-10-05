#include "Cat.h"
#include <queue>
#include <unordered_map>
#include <utility>
#include "World.h"
#include <stdexcept>

//define the queue to use
using WeightCell = std::pair<float, Point2D>;
using DjikstraQueue = std::priority_queue<WeightCell, std::vector<WeightCell>, std::greater<WeightCell>>;
using CameFrom = std::unordered_map<Point2D, Point2D>;
using CostSoFar = std::unordered_map<Point2D, float>;

Point2D Cat::Move(CatWorld* world) {
  auto rand = Random::Range(0, 5);
  auto pos = world->getCat();

  std::cout << pos.x << pos.y << std::endl;

  //initial setup
  DjikstraQueue queue = DjikstraQueue();
  queue.push({0, pos});
  CameFrom cameFrom = CameFrom();
  CostSoFar costSoFar = CostSoFar();
  cameFrom.emplace(pos, pos);
  costSoFar.emplace(pos, 0.0f);

  switch (rand) {
    case 0:
      return CatWorld::NE(pos);
    case 1:
      return CatWorld::NW(pos);
    case 2:
      return CatWorld::E(pos);
    case 3:
      return CatWorld::W(pos);
    case 4:
      return CatWorld::SW(pos);
    case 5:
      return CatWorld::SE(pos);
    default:
      throw std::runtime_error("random out of range");
  }
}
