#ifndef AGENT_H
#define AGENT_H

#include <glm/glm.hpp>
#include <functional>
#include <queue>
#include <unordered_map>
#include <vector>

class CatWorld;

enum class WorldEdges
{
    UP = 0,
    DOWN = 1,
    LEFT = 2,
    RIGHT = 3
};

// Point2D is now glm::ivec2 — same x,y interface, no OOP wrapper needed.
using Point2D = glm::ivec2;

//define the queue to use
struct WeightCell
{
  WeightCell(float weight, Point2D pos) { mWeight = weight; mPos = pos; }

  float mWeight;
  Point2D mPos;
};

static bool operator>(const WeightCell& lhs, const WeightCell& rhs) { return lhs.mWeight > rhs.mWeight; }

//define structures for use in algorithms
using DjikstraQueue = std::priority_queue<WeightCell, std::vector<WeightCell>, std::greater<WeightCell>>;
using CameFrom = std::unordered_map<Point2D, Point2D>;
using CostSoFar = std::unordered_map<Point2D, float>;

int getMinDistanceToEdge(const Point2D& pos, CatWorld* world);

// Hash specialization so Point2D (= glm::ivec2) works in unordered containers.
namespace std {
  template <> struct hash<glm::ivec2> {
    std::size_t operator()(const glm::ivec2& v) const noexcept {
      std::size_t seed = std::hash<int>{}(v.x);
      seed ^= std::hash<int>{}(v.y) + 0x9e3779b9u + (seed << 6) + (seed >> 2);
      return seed;
    }
  };
}  // namespace std



class Agent {
public:
  explicit Agent() = default;
  virtual ~Agent() = default;

  virtual Point2D Move(CatWorld*) = 0;

  std::vector<Point2D> generatePath(CatWorld* w);
};

#endif  // AGENT_H
