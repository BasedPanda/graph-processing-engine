#include <gtest/gtest.h>
#include "../src/core/graph.hpp"

namespace graph_engine {
namespace testing {

class GraphTest : public ::testing::Test {
protected:
    void SetUp() override {
        directed_graph_ = std::make_unique<Graph>(GraphProperties::DIRECTED);
        undirected_graph_ = std::make_unique<Graph>(GraphProperties::DIRECTED | GraphProperties::WEIGHTED);
    }

    std::unique_ptr<Graph> directed_graph_;
    std::unique_ptr<Graph> undirected_graph_;
};

TEST_F(GraphTest, CreateEmptyGraph) {
    EXPECT_EQ(directed_graph_->getVertexCount(), 0);
    EXPECT_EQ(directed_graph_->getEdgeCount(), 0);
    EXPECT_TRUE(directed_graph_->isDirected());
}

TEST_F(GraphTest, AddVertices) {
    vertex_id_t v1 = directed_graph_->addVertex();
    vertex_id_t v2 = directed_graph_->addVertex("test");
    
    EXPECT_EQ(directed_graph_->getVertexCount(), 2);
    EXPECT_EQ(directed_graph_->getVertex(v1)->getLabel(), "");
    EXPECT_EQ(directed_graph_->getVertex(v2)->getLabel(), "test");
}

TEST_F(GraphTest, AddEdges) {
    vertex_id_t v1 = directed_graph_->addVertex();
    vertex_id_t v2 = directed_graph_->addVertex();
    
    edge_id_t e1 = directed_graph_->addEdge(v1, v2);
    EXPECT_EQ(directed_graph_->getEdgeCount(), 1);
    
    const Edge* edge = directed_graph_->getEdge(e1);
    EXPECT_EQ(edge->getSource(), v1);
    EXPECT_EQ(edge->getTarget(), v2);
}

TEST_F(GraphTest, RemoveVertex) {
    vertex_id_t v1 = directed_graph_->addVertex();
    vertex_id_t v2 = directed_graph_->addVertex();
    vertex_id_t v3 = directed_graph_->addVertex();
    
    directed_graph_->addEdge(v1, v2);
    directed_graph_->addEdge(v2, v3);
    directed_graph_->addEdge(v3, v1);
    
    directed_graph_->removeVertex(v2);
    EXPECT_EQ(directed_graph_->getVertexCount(), 2);
    EXPECT_EQ(directed_graph_->getEdgeCount(), 1);
    EXPECT_TRUE(directed_graph_->hasEdge(v3, v1));
}

TEST_F(GraphTest, RemoveEdge) {
    vertex_id_t v1 = directed_graph_->addVertex();
    vertex_id_t v2 = directed_graph_->addVertex();
    
    edge_id_t e1 = directed_graph_->addEdge(v1, v2);
    EXPECT_TRUE(directed_graph_->hasEdge(v1, v2));
    
    directed_graph_->removeEdge(e1);
    EXPECT_FALSE(directed_graph_->hasEdge(v1, v2));
}

TEST_F(GraphTest, VertexProperties) {
    vertex_id_t v1 = directed_graph_->addVertex();
    Vertex* vertex = directed_graph_->getVertex(v1);
    
    vertex->setProperty("color", "red");
    EXPECT_TRUE(vertex->hasProperty("color"));
    EXPECT_EQ(vertex->getProperty("color"), "red");
}

TEST_F(GraphTest, EdgeProperties) {
    vertex_id_t v1 = directed_graph_->addVertex();
    vertex_id_t v2 = directed_graph_->addVertex();
    
    edge_id_t e1 = undirected_graph_->addEdge(v1, v2, 2.5f);
    Edge* edge = undirected_graph_->getEdge(e1);
    
    EXPECT_EQ(edge->getWeight(), 2.5f);
    edge->setProperty("type", "connection");
    EXPECT_TRUE(edge->hasProperty("type"));
}

TEST_F(GraphTest, GetNeighbors) {
    vertex_id_t v1 = directed_graph_->addVertex();
    vertex_id_t v2 = directed_graph_->addVertex();
    vertex_id_t v3 = directed_graph_->addVertex();
    
    directed_graph_->addEdge(v1, v2);
    directed_graph_->addEdge(v1, v3);
    
    auto neighbors = directed_graph_->getNeighbors(v1);
    EXPECT_EQ(neighbors.size(), 2);
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), v2) != neighbors.end());
    EXPECT_TRUE(std::find(neighbors.begin(), neighbors.end(), v3) != neighbors.end());
}

TEST_F(GraphTest, DirectedVsUndirected) {
    vertex_id_t v1 = directed_graph_->addVertex();
    vertex_id_t v2 = directed_graph_->addVertex();
    vertex_id_t v3 = undirected_graph_->addVertex();
    vertex_id_t v4 = undirected_graph_->addVertex();
    
    directed_graph_->addEdge(v1, v2);
    undirected_graph_->addEdge(v3, v4);
    
    EXPECT_TRUE(directed_graph_->hasEdge(v1, v2));
    EXPECT_FALSE(directed_graph_->hasEdge(v2, v1));
    
    EXPECT_TRUE(undirected_graph_->hasEdge(v3, v4));
    EXPECT_TRUE(undirected_graph_->hasEdge(v4, v3));
}

} // namespace testing
} // namespace graph_engine