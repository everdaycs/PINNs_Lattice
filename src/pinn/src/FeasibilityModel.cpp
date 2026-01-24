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

} // namespace pinn
