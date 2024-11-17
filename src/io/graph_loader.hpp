#pragma once

#include "../core/graph.hpp"
#include <string>
#include <memory>
#include <vector>
#include <fstream>
#include <sstream>

namespace graph_engine {

enum class FileFormat {
    EDGE_LIST,      // Simple edge list format: source target [weight]
    ADJACENCY_LIST, // Adjacency list format: vertex neighbor1 neighbor2...
    CSR,           // Compressed Sparse Row format
    DIMACS,        // DIMACS format for graph problems
    MTX            // Matrix Market format
};

class GraphLoader {
public:
    // Constructor
    GraphLoader();

    // Main loading methods
    std::unique_ptr<Graph> loadFromFile(const std::string& filename, 
                                      FileFormat format,
                                      GraphProperties properties = GraphProperties::DIRECTED);

    // Save methods
    void saveToFile(const Graph* graph, 
                   const std::string& filename, 
                   FileFormat format) const;

    // Configuration methods
    void setSkipComments(bool skip) { skip_comments_ = skip; }
    void setCommentChar(char c) { comment_char_ = c; }
    void setDelimiter(char d) { delimiter_ = d; }
    void setHeaderLines(int lines) { header_lines_ = lines; }

private:
    bool skip_comments_;
    char comment_char_;
    char delimiter_;
    int header_lines_;

    // Helper methods for loading different formats
    std::unique_ptr<Graph> loadEdgeList(std::ifstream& file, GraphProperties properties);
    std::unique_ptr<Graph> loadAdjacencyList(std::ifstream& file, GraphProperties properties);
    std::unique_ptr<Graph> loadCSR(std::ifstream& file, GraphProperties properties);
    std::unique_ptr<Graph> loadDIMACS(std::ifstream& file, GraphProperties properties);
    std::unique_ptr<Graph> loadMTX(std::ifstream& file, GraphProperties properties);

    // Helper methods for saving different formats
    void saveEdgeList(const Graph* graph, std::ofstream& file) const;
    void saveAdjacencyList(const Graph* graph, std::ofstream& file) const;
    void saveCSR(const Graph* graph, std::ofstream& file) const;
    void saveDIMACS(const Graph* graph, std::ofstream& file) const;
    void saveMTX(const Graph* graph, std::ofstream& file) const;

    // Utility methods
    bool isCommentLine(const std::string& line) const;
    void skipHeaderLines(std::ifstream& file) const;
    std::vector<std::string> splitLine(const std::string& line) const;
    void validateFile(const std::string& filename, bool writing = false) const;
};

} // namespace graph_engine