#include <gtest/gtest.h>
#include "../src/algorithms/pagerank.hpp"
#include "../src/algorithms/bfs.hpp"
#include <cmath>

namespace graph_engine {
namespace testing {

class AlgorithmTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create a simple test graph
        graph_ = std::make_unique<Graph>(GraphProperties::DIRECTED);
        
        // Add vertices
        v0_ = graph_->addVertex();
        v1_ = graph_->addVertex();
        v2_ = graph_->addVertex();
        v3_ = graph_->addVertex();
        
        // Add edges to form a simple cycle with a branch
        graph_->addEdge(v0_, v1_);
        graph_->addEdge(v1_, v2_);
        graph_->addEdge(v2_, v0_);
        graph_->addEdge(v1_, v3_);
    }

    std::unique_ptr<Graph> graph_;
    vertex_id_t v0_, v1_, v2_, v3_;
    const float FLOAT_TOLERANCE = 1e-6f;
};

TEST_F(AlgorithmTest, PageRankBasic) {
    PageRank pagerank(graph_.get());
    pagerank.setDampingFactor(0.85f);
    pagerank.setMaxIterations(100);
    pagerank.setTolerance(1e-6f);
    
    pagerank.execute();
    
    // Get results
    const auto& ranks = pagerank.getAllRanks();
    EXPECT_EQ(ranks.size(), graph_->getVertexCount());
    
    // Sum of all ranks should be approximately 1
    float sum = 0.0f;
    for (float rank : ranks) {
        sum += rank;
    }
    EXPECT_NEAR(sum, 1.0f, FLOAT_TOLERANCE);
    
    // v1 should have highest rank due to most connections
    float max_rank = 0.0f;
    vertex_id_t max_rank_vertex = 0;
    for (vertex_id_t i = 0; i < ranks.size(); ++i) {
        if (ranks[i] > max_rank) {
            max_rank = ranks[i];
            max_rank_vertex = i;
        }
    }
    EXPECT_EQ(max_rank_vertex, v1_);
}

TEST_F(AlgorithmTest, PageRankParameterValidation) {
    PageRank pagerank(graph_.get());
    
    EXPECT_THROW(pagerank.setDampingFactor(-0.1f), std::invalid_argument);
    EXPECT_THROW(pagerank.setDampingFactor(1.1f), std::invalid_argument);
    EXPECT_THROW(pagerank.setMaxIterations(0), std::invalid_argument);
    EXPECT_THROW(pagerank.setTolerance(-1e-6f), std::invalid_argument);
}

TEST_F(AlgorithmTest, BFSBasic) {
    BFS bfs(graph_.get());
    bfs.setStartVertex(v0_);
    bfs.execute();
    
    // Check distances
    EXPECT_EQ(bfs.getDistance(v0_), 0);
    EXPECT_EQ(bfs.getDistance(v1_), 1);
    EXPECT_EQ(bfs.getDistance(v2_), 2);
    EXPECT_EQ(bfs.getDistance(v3_), 2);
}

TEST_F(AlgorithmTest, BFSPaths) {
    BFS bfs(graph_.get());
    bfs.setStartVertex(v0_);
    bfs.execute();
    
    // Check path to v3
    auto path = bfs.getPath(v3_);
    EXPECT_EQ(path.size(), 3);
    EXPECT_EQ(path[0], v0_);
    EXPECT_EQ(path[1], v1_);
    EXPECT_EQ(path[2], v3_);
}

TEST_F(AlgorithmTest, BFSDisconnectedGraph) {
    // Add an isolated vertex
    vertex_id_t v4 = graph_->addVertex();
    
    BFS bfs(graph_.get());
    bfs.setStartVertex(v0_);
    bfs.execute();
    
    EXPECT_EQ(bfs.getDistance(v4), BFS::INFINITY_DISTANCE);
    EXPECT_TRUE(bfs.getPath(v4).empty());
}

TEST_F(AlgorithmTest, AlgorithmExecution) {
    // Test both CPU and GPU execution
    std::vector<ExecutionDevice> devices = {
        ExecutionDevice::CPU,
        ExecutionDevice::GPU
    };
    
    for (auto device : devices) {
        PageRank pagerank(graph_.get());
        pagerank.execute(device);
        
        const auto& ranks = pagerank.getAllRanks();
        EXPECT_EQ(ranks.size(), graph_->getVertexCount());
        
        float sum = 0.0f;
        for (float rank : ranks) {
            sum += rank;
        }
        EXPECT_NEAR(sum, 1.0f, FLOAT_TOLERANCE);
    }
}

} // namespace testing
} // namespace graph_engine