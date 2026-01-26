#include "pinn/FeasibilityModel.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace pinn {

FeasibilityModel::FeasibilityModel() : loaded_(false) {}

bool FeasibilityModel::load(const std::string& model_path, const std::string& norm_path) {
    try {
        // Load TorchScript
        module_ = torch::jit::load(model_path);
        module_.eval();
        
        // Load Norms
        std::ifstream f(norm_path);
        if (!f.is_open()) return false;
        
        json j;
        f >> j;
        
        norms_.clear();
        for (const auto& item : j["features"]) {
            norms_.push_back({
                item["name"].get<std::string>(),
                item["mean"].get<float>(),
                item["std"].get<float>()
            });
        }
        
        loaded_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading model or norms: " << e.what() << std::endl;
        return false;
    }
}

float FeasibilityModel::infer(const std::vector<float>& features) {
    if (!loaded_ || features.size() != norms_.size()) {
        return 999.0f;
    }
    
    // Normalize
    std::vector<float> normed_feats;
    normed_feats.reserve(features.size());
    for (size_t i = 0; i < features.size(); ++i) {
        if (norms_[i].std > 1e-6) {
            normed_feats.push_back((features[i] - norms_[i].mean) / norms_[i].std);
        } else {
            normed_feats.push_back(features[i] - norms_[i].mean);
        }
    }
    
    // Inference
    try {
        std::vector<int64_t> sizes = {1, static_cast<int64_t>(features.size())};
        at::Tensor input = torch::from_blob(normed_feats.data(), sizes, torch::kFloat32).clone();
        
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input);
        
        at::Tensor output = module_.forward(inputs).toTensor();
        return output.item<float>();
    } catch (const std::exception& e) {
        std::cerr << "Inference error: " << e.what() << std::endl;
        return 999.0f;
    }
}

std::vector<float> FeasibilityModel::inferBatch(const std::vector<float>& features, size_t batch_size) {
    if (!loaded_) return std::vector<float>(batch_size, 999.0f);
    
    try {
        size_t feat_dim = features.size() / batch_size;
        std::vector<float> normed_feats = features;
        for (size_t i = 0; i < batch_size; ++i) {
            for (size_t j = 0; j < feat_dim; ++j) {
                normed_feats[i * feat_dim + j] = (features[i * feat_dim + j] - norms_[j].mean) / norms_[j].std;
            }
        }

        std::vector<int64_t> sizes = {static_cast<int64_t>(batch_size), static_cast<int64_t>(feat_dim)};
        at::Tensor input = torch::from_blob(normed_feats.data(), sizes, torch::kFloat32).clone();
        
        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(input);
        
        at::Tensor output = module_.forward(inputs).toTensor().reshape({-1});
        auto data_ptr = output.data_ptr<float>();
        return std::vector<float>(data_ptr, data_ptr + batch_size);
    } catch (const std::exception& e) {
        std::cerr << "Batch inference error: " << e.what() << std::endl;
        return std::vector<float>(batch_size, 999.0f);
    }
}

} // namespace pinn
