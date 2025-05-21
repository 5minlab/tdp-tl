// example: Foo<T> is a header-only C++ template.
// We only need Foo<int>::bar and Foo<double>::baz
// #include "foo.hpp"

#include <nanovdb/tools/CreateNanoGrid.h>
#include <nanovdb/tools/GridBuilder.h>

using namespace nanovdb;
using namespace nanovdb::tools;

typedef uint8_t ValueType;

std::vector<build::LeafNode<ValueType> *> leafs;
std::vector<nanovdb::Coord> coords;

extern "C" {
intptr_t grid_new() {
  auto buildGrid = new build::Grid<ValueType>(0);
  return (intptr_t)buildGrid;
}

void grid_delete(intptr_t grid) {
  build::Grid<ValueType> *buildGrid = (build::Grid<ValueType> *)grid;
  delete buildGrid;
}

ValueType get_value(intptr_t grid, int32_t x, int32_t y, int32_t z) {
  build::Grid<ValueType> *buildGrid = (build::Grid<ValueType> *)grid;
  auto coord = nanovdb::Coord(x, y, z);
  return buildGrid->tree().getValue(coord);
}

ValueType add_value(intptr_t grid, int32_t x, int32_t y, int32_t z,
                    ValueType value) {
  build::Grid<ValueType> *buildGrid = (build::Grid<ValueType> *)grid;
  auto coord = nanovdb::Coord(x, y, z);
  value += buildGrid->tree().getValue(coord);
  buildGrid->tree().setValue(coord, value);
  return value;
}

void set_value(intptr_t grid, int32_t x, int32_t y, int32_t z,
               ValueType value) {
  build::Grid<ValueType> *buildGrid = (build::Grid<ValueType> *)grid;
  auto coord = nanovdb::Coord(x, y, z);
  buildGrid->tree().setValue(coord, value);
}

size_t iter_init(intptr_t grid) {
  auto buildGrid = (build::Grid<ValueType> *)grid;
  leafs.clear();

  using Node0 = build::LeafNode<ValueType>;
  using Node1 = build::InternalNode<Node0>;
  using Node2 = build::InternalNode<Node1>;

  size_t count = 0;
  for (auto it2 = buildGrid->tree().root().cbeginChildOn(); it2; ++it2) {
    Node2 &upper = const_cast<Node2 &>(*it2);
    for (auto it1 = upper.cbeginChildOn(); it1; ++it1) {
      Node1 &lower = const_cast<Node1 &>(*it1);
      for (auto it0 = lower.cbeginChildOn(); it0; ++it0) {
        Node0 &leaf = const_cast<Node0 &>(*it0);
        leafs.push_back(&leaf);
        count++;
      }
    }
  }

  return count;
}

void iter_get0(size_t idx, int32_t *coord) {
  auto leaf = leafs[idx];
  auto origin = leaf->mOrigin;
  coord[0] = origin[0];
  coord[1] = origin[1];
  coord[2] = origin[2];
}

void iter_get(size_t idx, int32_t *coord, ValueType *buf) {
  auto leaf = leafs[idx];
  auto origin = leaf->mOrigin;
  coord[0] = origin[0];
  coord[1] = origin[1];
  coord[2] = origin[2];
  memcpy(buf, leafs[idx]->mValues, sizeof(leafs[idx]->mValues));
}

size_t iter2_init(intptr_t grid) {
  auto buildGrid = (build::Grid<ValueType> *)grid;
  coords.clear();

  using Node0 = build::LeafNode<ValueType>;
  using Node1 = build::InternalNode<Node0>;
  using Node2 = build::InternalNode<Node1>;

  size_t count = 0;
  for (auto it2 = buildGrid->tree().root().cbeginChildOn(); it2; ++it2) {
    Node2 &upper = const_cast<Node2 &>(*it2);
    for (auto it1 = upper.cbeginChildOn(); it1; ++it1) {
      Node1 &lower = const_cast<Node1 &>(*it1);
      coords.push_back(lower.mOrigin);
      count += 1;
    }
  }

  return count;
}

void iter2_get0(size_t idx, int32_t *coord) {
  auto coord2 = coords[idx];
  coord[0] = coord2[0];
  coord[1] = coord2[1];
  coord[2] = coord2[2];
}
}
