#include "graph_loader.hpp"
#include <stdexcept>
#include <algorithm>
#include <filesystem>
#include <unordered_set>
#include <sstream>
#include <cstring>

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
    validateFile(filename, false);
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

        try {
            vertex_id_t source = std::stoull(tokens[0]);
            vertex_id_t target = std::stoull(tokens[1]);
            
            vertices.insert(source);
            vertices.insert(target);
        } catch (const std::exception& e) {
            throw std::runtime_error("Error parsing edge list: " + std::string(e.what()));
        }
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

        try {
            vertex_id_t source = std::stoull(tokens[0]);
            vertex_id_t target = std::stoull(tokens[1]);
            
            if (tokens.size() >= 3 && (properties & GraphProperties::WEIGHTED)) {
                weight_t weight = std::stof(tokens[2]);
                graph->addEdge(source, target, weight);
            } else {
                graph->addEdge(source, target);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error("Error creating edge: " + std::string(e.what()));
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

        try {
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
        } catch (const std::exception& e) {
            throw std::runtime_error("Error parsing adjacency list: " + std::string(e.what()));
        }
    }

    return graph;
}

std::unique_ptr<Graph> GraphLoader::loadCSR(std::ifstream& file, GraphProperties properties) {
    auto graph = std::make_unique<Graph>(properties);
    std::string line;

    // Read number of vertices and edges
    if (!std::getline(file, line)) {
        throw std::runtime_error("Failed to read CSR header");
    }

    auto header = splitLine(line);
    if (header.size() < 2) {
        throw std::runtime_error("Invalid CSR format: missing header information");
    }

    try {
        size_t num_vertices = std::stoull(header[0]);
        size_t num_edges = std::stoull(header[1]);

        // Read row pointers
        if (!std::getline(file, line)) {
            throw std::runtime_error("Failed to read row pointers");
        }
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
        if (!std::getline(file, line)) {
            throw std::runtime_error("Failed to read column indices");
        }
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
    } catch (const std::exception& e) {
        throw std::runtime_error("Error parsing CSR format: " + std::string(e.what()));
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
            try {
                num_vertices = std::stoull(tokens[2]);
                num_edges = std::stoull(tokens[3]);
                break;
            } catch (const std::exception& e) {
                throw std::runtime_error("Error parsing DIMACS header: " + std::string(e.what()));
            }
        }
    }

    if (num_vertices == 0) {
        throw std::runtime_error("Invalid DIMACS format: missing or invalid header");
    }

    // Create vertices
    for (vertex_id_t i = 1; i <= num_vertices; ++i) {
        graph->addVertex(std::to_string(i));
    }

    // Read edges
    size_t edge_count = 0;
    while (std::getline(file, line) && edge_count < num_edges) {
        if (line.empty() || (skip_comments_ && isCommentLine(line))) continue;

        auto tokens = splitLine(line);
        if (tokens.empty() || tokens[0] != "e") continue;

        if (tokens.size() < 3) {
            throw std::runtime_error("Invalid DIMACS format: incomplete edge line");
        }

        try {
            vertex_id_t source = std::stoull(tokens[1]) - 1; // DIMACS uses 1-based indexing
            vertex_id_t target = std::stoull(tokens[2]) - 1;

            if (tokens.size() >= 4 && (properties & GraphProperties::WEIGHTED)) {
                weight_t weight = std::stof(tokens[3]);
                graph->addEdge(source, target, weight);
            } else {
                graph->addEdge(source, target);
            }
            edge_count++;
        } catch (const std::exception& e) {
            throw std::runtime_error("Error parsing DIMACS edge: " + std::string(e.what()));
        }
    }

    if (edge_count != num_edges) {
        throw std::runtime_error("Invalid DIMACS format: edge count mismatch");
    }

    return graph;
}

std::unique_ptr<Graph> GraphLoader::loadMTX(std::ifstream& file, GraphProperties properties) {
    auto graph = std::make_unique<Graph>(properties);
    std::string line;

    // Skip header until %%MatrixMarket line
    bool found_header = false;
    while (std::getline(file, line)) {
        if (line.substr(0, 14) == "%%MatrixMarket") {
            found_header = true;
            break;
        }
    }

    if (!found_header) {
        throw std::runtime_error("Invalid MTX format: missing header");
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

    try {
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
    } catch (const std::exception& e) {
        throw std::runtime_error("Error parsing MTX format: " + std::string(e.what()));
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
    const size_t num_vertices = graph->getVertexCount();
    const size_t num_edges = graph->getEdgeCount();

    // Write header
    file << num_vertices << delimiter_ << num_edges << '\n';

    // Calculate and write row pointers
    std::vector<size_t> row_ptr(num_vertices + 1, 0);
    std::vector<vertex_id_t> col_idx;
    col_idx.reserve(num_edges);

    size_t current_edge = 0;
    for (vertex_id_t i = 0; i < num_vertices; ++i) {
        row_ptr[i] = current_edge;
        const Vertex* vertex = graph->getVertex(i);
        if (vertex) {
            for (const auto& [target, _] : vertex->getOutEdges()) {
                col_idx.push_back(target);
                current_edge++;
            }
        }
    }
    row_ptr[num_vertices] = current_edge;

    // Write row pointers
    for (size_t i = 0; i < row_ptr.size(); ++i) {
        if (i > 0) file << delimiter_;
        file << row_ptr[i];
    }
    file << '\n';

    // Write column indices
    for (size_t i = 0; i < col_idx.size(); ++i) {
        if (i > 0) file << delimiter_;
        file << col_idx[i];
    }
    file << '\n';
}

void GraphLoader::saveDIMACS(const Graph* graph, std::ofstream& file) const {
    // Write header
    file << "p edge " << graph->getVertexCount() << " " << graph->getEdgeCount() << "\n";

    // Write edges
    for (const auto& [_, edge_ptr] : graph->edges()) {
        const Edge* edge = edge_ptr.get();
        file << "e " << (edge->getSource() + 1) << " " // DIMACS uses 1-based indexing
             << (edge->getTarget() + 1);
        if (graph->isWeighted()) {
            file << " " << edge->getWeight();
        }
        file << '\n';
    }
}

void GraphLoader::saveMTX(const Graph* graph, std::ofstream& file) const {
    // Write MTX header
    file << "%%MatrixMarket matrix coordinate real general\n";
    file << "% Generated by Graph Processing Engine\n";
    
    // Write dimensions and number of edges
    file << graph->getVertexCount() << " " 
         << graph->getVertexCount() << " " 
         << graph->getEdgeCount() << "\n";

    // Write edges
    for (const auto& [_, edge_ptr] : graph->edges()) {
        const Edge* edge = edge_ptr.get();
        file << (edge->getSource() + 1) << " "  // MTX uses 1-based indexing
             << (edge->getTarget() + 1);
        if (graph->isWeighted()) {
            file << " " << edge->getWeight();
        }
        file << '\n';
    }
}

bool GraphLoader::isCommentLine(const std::string& line) const {
    return !line.empty() && line[0] == comment_char_;
}

void GraphLoader::skipHeaderLines(std::ifstream& file) const {
    std::string line;
    for (int i = 0; i < header_lines_; ++i) {
        if (!std::getline(file, line)) {
            throw std::runtime_error("Not enough header lines to skip");
        }
    }
}

std::vector<std::string> GraphLoader::splitLine(const std::string& line) const {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    
    try {
        while (std::getline(ss, token, delimiter_)) {
            if (!token.empty()) {
                tokens.push_back(token);
            }
        }
    } catch (const std::exception& e) {
        throw std::runtime_error("Error parsing line: " + line + "\nError: " + e.what());
    }
    
    return tokens;
}

void GraphLoader::validateFile(const std::string& filename, bool writing) const {
    std::filesystem::path path(filename);
    
    if (writing) {
        auto parent_path = path.parent_path();
        if (!parent_path.empty() && !std::filesystem::exists(parent_path)) {
            throw std::runtime_error("Directory does not exist: " + parent_path.string());
        }
        return;
    }
    
    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("File does not exist: " + filename);
    }
    
    if (std::filesystem::file_size(path) == 0) {
        throw std::runtime_error("File is empty: " + filename);
    }

    // Check file permissions
    try {
        std::ifstream test(filename);
        if (!test.is_open()) {
            throw std::runtime_error("Cannot open file: " + filename);
        }
        test.close();
    } catch (const std::exception& e) {
        throw std::runtime_error("File access error: " + std::string(e.what()));
    }
}

} // namespace graph_engine