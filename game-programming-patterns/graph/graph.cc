#include "graph.h"

Graph::Graph(const char *filename, bool directed = false) {
  directed_ = directed;
  int a, b;
  // 默认能打开，如果想安全，使用if (!infile.is_open())作进一步处理
  std::ifstream infile(filename, std::ios_base::in);
  // 节点和边数量
  infile >> a >> b;
  vcount_ = a;
  ecount_ = b;
  vertices_.resize(vcount_);
  // 读取边
  for (int i = 0; i < ecount_; ++i) {
    infile >> a >> b;
    int v = a;
    int w = b;
    vertices_[v].insert(w);
    if (!directed_) {
      vertices_[w].insert(v);
    }
  }
  infile.close();
}

// 添加顶点
int Graph::AddVertex() {
  std::set<int> temp;
  vertices_.push_back(temp);
  ++vcount_;
  return 0;
}

// 删除顶点
int Graph::RemoveVertex(const int &u) {
  if (u > vertices_.size()) {
    return -1;
  }
  // 遍历图，寻找与顶点的相关的边
  // 无向图，有关的边一定在该顶点的邻接关系中
  if (!directed_) {
    int e = vertices_[u].size();
    vertices_.erase(vertices_.begin() + u);
    ecount_ -= e;
    --vcount_;
    return 0;
  } else {
    // 遍历图
    for (int i = 0; i < vertices_.size(); ++i) {
      RemoveEdge(i, u);
    }
    vertices_.erase(vertices_.begin() + u);
    --vcount_;
    return 0;
  }
  return -1;
}

// 添加边
int Graph::AddEdge(const int &u, const int &v) {
  // 不绑安全带，使用需谨慎
  vertices_[u].insert(v);
  if (!directed_) {
    vertices_[v].insert(u);
  }
  ++ecount_;
  return 0;
}

// 删除边
int Graph::RemoveEdge(const int &u, const int &v) {
  auto it_find = vertices_[u].find(v);
  if (it_find != vertices_[u].end()) {
    vertices_[u].erase(v);
    --ecount_;
  } else {
    return -1;
  }
  if (directed_) { return 0; }
  // 无向图删除反向边
  it_find = vertices_[v].find(u);
  if (it_find != vertices_[u].end()) {
    vertices_[v].erase(u);
  } else {
    // 人和人之间的信任呢？
    return -1;
  }
  return 0;
}

// 检查两个顶点之间是否有邻接关系
bool Graph::IsAdjacent(const int &u, const int &v) {
  if (vertices_[u].count(v) == 1) {
    return true;
  }
  return false;
}
// 打印图
void PrintGraph(const Graph &graph) {
  for (int i = 0; i < graph.vcount(); i++) {
    std::cout << i << " -->";
    for (auto it = graph.cbegin(i); it != graph.cend(i); ++it) {
      std::cout << " " << *it;
    }
    std::cout << std::endl;
  }
}