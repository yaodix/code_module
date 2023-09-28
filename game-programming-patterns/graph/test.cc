#include "cxxgraph/include/CXXGraph.hpp"

int main() {
  CXXGraph::Node<int> node1("1", 1);
  CXXGraph::Node<int> node2("2", 2);
  std::pair<const CXXGraph::Node<int> *, const CXXGraph::Node<int> *> pairNode(
      &node1, &node2);
  CXXGraph::Edge<int> edge(1, pairNode);
  CXXGraph::T_EdgeSet<int> edgeSet;
  edgeSet.insert(make_shared<CXXGraph::Edge<int>>(edge));
  CXXGraph::Graph<int> graph(edgeSet);

  graph.writeToFile("./g.txt");
  
  return 0;
}