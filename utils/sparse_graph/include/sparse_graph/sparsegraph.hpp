#ifndef SPARSEGRAPH_HPP
#define SPARSEGRAPH_HPP

#include <memory>
#include <unordered_set>
#include <vector>

namespace mocka
{

template <class NodeData, class EdgeData>
class SGEdge;

template <class NodeData, class EdgeData>
class SGNode
{
public:
  typedef std::shared_ptr<SGEdge<NodeData, EdgeData> > Edge;

public:
  SGNode()
  {
  }

  SGNode(const NodeData& d)
    : data(d)
  {
  }

public:
  void addEdge(Edge e);
  void removeEdge(Edge e);

public:
  NodeData                 data;
  std::unordered_set<Edge> outEdge;
}; // sparse graph node

template <class NodeData, class EdgeData>
class SGEdge
{
public:
  typedef std::shared_ptr<SGNode<NodeData, EdgeData> > Node;

public:
  SGEdge()
  {
  }

public:
  EdgeData data;

public:
  // getters
  Node getSink() const;
  Node getSource() const;

public:
  // setters;
  void setSink(const Node& value);
  void setSource(const Node& value);

public:
  // for convinent
  Node sink;
  Node source;
}; // sparse graph edge

template <class NodeData, class EdgeData>
class SparseGraph
{
public:
  typedef SGEdge<NodeData, EdgeData> _Edge;
  typedef SGNode<NodeData, EdgeData> _Node;
  typedef std::shared_ptr<_Edge> Edge;
  typedef std::shared_ptr<_Node> Node;

public:
  SparseGraph()
  {
  }

public:
  void clear();
  Edge addEdge(Node& source, Node& sink);
  void removeEdge(const Edge& e);
  void removeEdge(Node& source, Node& sink);

public:
  Edge getEdge(Node& source, Node& sink);

public:
  std::unordered_set<Node> nodes;
}; // SparseGraph

template <class NodeData, class EdgeData>
void
SparseGraph<NodeData, EdgeData>::clear()
{
}

template <class NodeData, class EdgeData>
typename SparseGraph<NodeData, EdgeData>::Edge
SparseGraph<NodeData, EdgeData>::addEdge(SparseGraph::Node& source,
                                         SparseGraph::Node& sink)
{
  nodes.insert(source);
  nodes.insert(sink);
  SparseGraph<NodeData, EdgeData>::Edge ans(
    new SparseGraph<NodeData, EdgeData>::_Edge());

  ans->source = source;
  ans->sink   = sink;

  source->outEdge.insert(ans);

  return ans;
}

template <class NodeData, class EdgeData>
void
SparseGraph<NodeData, EdgeData>::removeEdge(const SparseGraph::Edge& e)
{
  e->source->outEdge.erase(e);
}

template <class NodeData, class EdgeData>
typename SparseGraph<NodeData, EdgeData>::Edge
SparseGraph<NodeData, EdgeData>::getEdge(SparseGraph::Node& source,
                                         SparseGraph::Node& sink)
{
}

template <class NodeData, class EdgeData>
typename SGEdge<NodeData, EdgeData>::Node
SGEdge<NodeData, EdgeData>::getSink() const
{
  return sink;
}

template <class NodeData, class EdgeData>
void
SGEdge<NodeData, EdgeData>::setSink(const Node& value)
{
  sink = value;
}

template <class NodeData, class EdgeData>
typename SGEdge<NodeData, EdgeData>::Node
SGEdge<NodeData, EdgeData>::getSource() const
{
  return source;
}

template <class NodeData, class EdgeData>
void
SGEdge<NodeData, EdgeData>::setSource(const Node& value)
{
  source = value;
}

} // namespace mocka

#endif // SPARSEGRAPH_HPP
