#include "graph_loader.hpp"
#include <stdexcept>
#include <algorithm>
#include <filesystem>
#include <unordered_set>

namespace graph_engine {

GraphLoader::GraphLoader()
    : skip_comments_(true)
    , comment_char_('#')
    , delimiter_(' ')
    , header_lines_(0)
{}

std::unique_ptr<Graph> GraphLoader::loadFromFile(
    const std::string& filename,
    FileFormat format,
    GraphProperties properties)
{
    validateFile(filename);
    std::ifstream file(filename);

    skipHeaderLines(file);

    switch (format) {
        case FileFormat::EDGE_LIST:
            return loadEdgeList(file, properties);
        case FileFormat::ADJACENCY_LIST:
            return loadAdjacencyList(file, properties);
        case FileFormat::CSR:
            return loadCSR(file, properties);
        case FileFormat::DIMACS:
            return loadDIMACS(file, properties);
        case FileFormat::MTX:
            return loadMTX(file, properties);
        default:
            throw std::invalid_argument("Unsupported file format");
    }
}

void GraphLoader::saveToFile(
    const Graph* graph,
    const std::string& filename,
    FileFormat format) const
{
    if (!graph) {
        throw std::invalid_argument("Null graph pointer");
    }

    validateFile(filename, true);
    std::ofstream file(filename);

    switch (format) {
        case FileFormat::EDGE_LIST:
            saveEdgeList(graph, file);
            break;
        case FileFormat::ADJACENCY_LIST:
            saveAdjacencyList(graph, file);
            break;
        case FileFormat::CSR:
            saveCSR(graph, file);
            break;
        case FileFormat::DIMACS:
            saveDIMACS(graph, file);
            break;
        case FileFormat::MTX:
            saveMTX(graph, file);
            break;
        default:
            throw std::invalid_argument("Unsupported file format");
    }
}

std::unique_ptr<Graph> GraphLoader::loadEdgeList(std::ifstream& file, GraphProperties properties) {
    auto graph = std::make_unique<Graph>(properties);
    std::string line;
    std::unordered_set<vertex_id_t> vertices;

    // First pass: collect all unique vertices
    while (std::getline(file, line)) {
        if (skip_comments_ && isCommentLine(line)) continue;
        
        auto tokens = splitLine(line);
        if (tokens.size() < 2) continue;

        vertex_id_t source = std::stoull(tokens[0]);
        vertex_id_t target = std::stoull(tokens[1]);
        
        vertices.insert(source);
        vertices.insert(target);
    }

    // Create all vertices
    for (vertex_id_t v : vertices) {
        graph->addVertex(std::to_string(v));
    }

    // Second pass: add edges
    file.clear();
    file.seekg(0);
    skipHeaderLines(file);

    while (std::getline(file, line)) {
        if (skip_comments_ && isCommentLine(line)) continue;
        
        auto tokens = splitLine(line);
        if (tokens.size() < 2) continue;

        vertex_id_t source = std::stoull(tokens[0]);
        vertex_id_t target = std::stoull(tokens[1]);
        
        if (tokens.size() >= 3 && (properties & GraphProperties::WEIGHTED)) {
            weight_t weight = std::stof(tokens[2]);
            graph->addEdge(source, target, weight);
        } else {
            graph->addEdge(source, target);
        }
    }

    return graph;
}

std::unique_ptr<Graph> GraphLoader::loadAdjacencyList(std::ifstream& file, GraphProperties properties) {
    auto graph = std::make_unique<Graph>(properties);
    std::string line;
    
    while (std::getline(file, line)) {
        if (skip_comments_ && isCommentLine(line)) continue;
        
        auto tokens = splitLine(line);
        if (tokens.empty()) continue;

        // First token is the source vertex
        vertex_id_t source = std::stoull(tokens[0]);
        if (!graph->getVertex(source)) {
            graph->addVertex(std::to_string(source));
        }

        // Remaining tokens are neighbors
        for (size_t i = 1; i < tokens.size(); ++i) {
            vertex_id_t target = std::stoull(tokens[i]);
            if (!graph->getVertex(target)) {
                graph->addVertex(std::to_string(target));
            }
            graph->addEdge(source, target);
        }
    }

    return graph;
}

std::unique_ptr<Graph> GraphLoader::loadCSR(std::ifstream& file, GraphProperties properties) {
    auto graph = std::make_unique<Graph>(properties);
    std::string line;

    // Read number of vertices and edges
    std::getline(file, line);
    auto header = splitLine(line);
    if (header.size() < 2) {
        throw std::runtime_error("Invalid CSR format: missing header");
    }

    size_t num_vertices = std::stoull(header[0]);
    size_t num_edges = std::stoull(header[1]);

    // Read row pointers
    std::getline(file, line);
    auto row_ptr_tokens = splitLine(line);
    if (row_ptr_tokens.size() != num_vertices + 1) {
        throw std::runtime_error("Invalid CSR format: incorrect row pointer array size");
    }

    std::vector<size_t> row_ptr;
    row_ptr.reserve(row_ptr_tokens.size());
    for (const auto& token : row_ptr_tokens) {
        row_ptr.push_back(std::stoull(token));
    }

    // Read column indices
    std::getline(file, line);
    auto col_idx_tokens = splitLine(line);
    if (col_idx_tokens.size() != num_edges) {
        throw std::runtime_error("Invalid CSR format: incorrect column index array size");
    }

    // Create vertices
    for (vertex_id_t i = 0; i < num_vertices; ++i) {
        graph->addVertex(std::to_string(i));
    }

    // Create edges
    for (vertex_id_t i = 0; i < num_vertices; ++i) {
        for (size_t j = row_ptr[i]; j < row_ptr[i + 1]; ++j) {
            vertex_id_t target = std::stoull(col_idx_tokens[j]);
            graph->addEdge(i, target);
        }
    }

    return graph;
}

std::unique_ptr<Graph> GraphLoader::loadDIMACS(std::ifstream& file, GraphProperties properties) {
    auto graph = std::make_unique<Graph>(properties);
    std::string line;
    
    size_t num_vertices = 0;
    size_t num_edges = 0;

    // Read header
    while (std::getline(file, line)) {
        if (line.empty() || (skip_comments_ && isCommentLine(line))) continue;

        auto tokens = splitLine(line);
        if (tokens.empty()) continue;

        if (tokens[0] == "p") {
            if (tokens.size() < 4) {
                throw std::runtime_error("Invalid DIMACS format: incomplete problem line");
            }
            num_vertices = std::stoull(tokens[2]);
            num_edges = std::stoull(tokens[3]);
            break;
        }
    }

    // Create vertices
    for (vertex_id_t i = 1; i <= num_vertices; ++i) {
        graph->addVertex(std::to_string(i));
    }

    // Read edges
    while (std::getline(file, line)) {
        if (line.empty() || (skip_comments_ && isCommentLine(line))) continue;

        auto tokens = splitLine(line);
        if (tokens.empty() || tokens[0] != "e") continue;

        if (tokens.size() < 3) {
            throw std::runtime_error("Invalid DIMACS format: incomplete edge line");
        }

        vertex_id_t source = std::stoull(tokens[1]) - 1; // DIMACS uses 1-based indexing
        vertex_id_t target = std::stoull(tokens[2]) - 1;

        if (tokens.size() >= 4 && (properties & GraphProperties::WEIGHTED)) {
            weight_t weight = std::stof(tokens[3]);
            graph->addEdge(source, target, weight);
        } else {
            graph->addEdge(source, target);
        }
    }

    return graph;
}

std::unique_ptr<Graph> GraphLoader::loadMTX(std::ifstream& file, GraphProperties properties) {
    auto graph = std::make_unique<Graph>(properties);
    std::string line;

    // Skip header until %%MatrixMarket line
    while (std::getline(file, line)) {
        if (line.substr(0, 14) == "%%MatrixMarket") {
            break;
        }
    }

    // Skip comments
    while (std::getline(file, line)) {
        if (!isCommentLine(line)) break;
    }

    // Parse dimensions
    auto tokens = splitLine(line);
    if (tokens.size() < 3) {
        throw std::runtime_error("Invalid MTX format: missing dimensions");
    }

    size_t rows = std::stoull(tokens[0]);
    size_t cols = std::stoull(tokens[1]);
    
    // Create vertices
    for (vertex_id_t i = 0; i < std::max(rows, cols); ++i) {
        graph->addVertex(std::to_string(i));
    }

    // Read edges
    while (std::getline(file, line)) {
        if (skip_comments_ && isCommentLine(line)) continue;

        tokens = splitLine(line);
        if (tokens.size() < 2) continue;

        vertex_id_t source = std::stoull(tokens[0]) - 1; // MTX uses 1-based indexing
        vertex_id_t target = std::stoull(tokens[1]) - 1;

        if (tokens.size() >= 3 && (properties & GraphProperties::WEIGHTED)) {
            weight_t weight = std::stof(tokens[2]);
            graph->addEdge(source, target, weight);
        } else {
            graph->addEdge(source, target);
        }
    }

    return graph;
}

void GraphLoader::saveEdgeList(const Graph* graph, std::ofstream& file) const {
    for (const auto& [_, edge_ptr] : graph->edges()) {
        const Edge* edge = edge_ptr.get();
        if (graph->isWeighted()) {
            file << edge->getSource() << delimiter_ 
                 << edge->getTarget() << delimiter_ 
                 << edge->getWeight() << '\n';
        } else {
            file << edge->getSource() << delimiter_ 
                 << edge->getTarget() << '\n';
        }
    }
}

void GraphLoader::saveAdjacencyList(const Graph* graph, std::ofstream& file) const {
    for (const auto& [vertex_id, vertex_ptr] : graph->vertices()) {
        file << vertex_id;
        for (const auto& [neighbor_id, _] : vertex_ptr->getOutEdges()) {
            file << delimiter_ << neighbor_id;
        }
        file << '\n';
    }
}

void GraphLoader::saveCSR(const Graph* graph, std::ofstream& file) const {
    size_t num_vertices = graph->getVertexCount();
    size_t num_edges = graph->getEdgeCount();

    // Write header
    file << num_vertices << delimiter_ << num_edges << '\n';

    // Calculate row pointers
    std::vector<size_t> row_ptr(num_vertices + 1, 0);
    std::vector<vertex_id_t> col_idx;
    col_idx.reserve(num_edges);

    size_t current_ptr = 0;
    for (vertex_id_t i = 0; i < num_vertices; ++i) {
        row_ptr[i] = current_ptr;
        const Vertex* vertex = graph->getVertex(i);
        if (vertex) {
            for (const auto& [target, _] : vertex->getOutEdges()) {
                col_idx.push_back(target);
                current_ptr++;
            }
        }
    }
    row_ptr[num_vertices]