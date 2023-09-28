#include <iostream>
#include <vector>
#include <set>
#include <list>
#include <fstream>
#include <limits>
#include <queue>

// 邻接集合
typedef std::set<int> AdjSet;
// 邻接集
class Graph {
 protected:
  // 邻接表向量
  std::vector<AdjSet> vertices_;
  // 顶点数量
  int vcount_;
  // 边的数量
  int ecount_;
  bool directed_;
 public:
  Graph(bool directed = false)
    : ecount_(0), vcount_(0),
      vertices_(0), directed_(directed) {};
  Graph(int n, bool directed)
    : ecount_(0), vcount_(n),
      vertices_(n), directed_(directed) {};
  // 从文件中初始化
  Graph(const char *filename, bool directed);
  virtual ~Graph() {
    vertices_.clear();
    vcount_ = 0;
    ecount_ = 0;
  }
  // 取值函数
  virtual int vcount() const { return vcount_; };
  virtual int ecount() const { return ecount_; };
  virtual bool directed() const { return directed_; };
  // 某条边是否存在
  virtual bool IsAdjacent(const int &u, const int &v);
  // 约定：成功返回 0，不存在 -1，已存在 1
  // 添加边
  virtual int AddEdge(const int &u, const int &v);
  // 添加顶点
  virtual int AddVertex();
  // 删除边
  virtual int RemoveEdge(const int &u, const int &v);
  // 删除顶点
  virtual int RemoveVertex(const int &u);
  // 返回顶点的邻接集
  virtual std::set<int>& Adj(const int &u) { return vertices_[u]; }
  // 迭代器
  virtual AdjSet::const_iterator begin(const int u) { return vertices_[u].begin(); };
  virtual AdjSet::const_iterator end(const int u) { return vertices_[u].end(); };
  virtual AdjSet::const_iterator cbegin(const int u) const { return vertices_[u].cbegin(); };
  virtual AdjSet::const_iterator cend(const int u) const { return vertices_[u].cend(); };
}; // class Graph


