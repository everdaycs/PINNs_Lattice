#include "pinn/SpeedCorrectorModel.hpp"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <algorithm> // for clamp

using json = nlohmann::json;

namespace pinn {

SpeedCorrectorModel::SpeedCorrectorModel() {}

bool SpeedCorrectorModel::load(const std::string& model_path, 
                               const std::string& norm_path,
                               const std::string& meta_path) {
    try {
        // 1. Load Torch Model
        module_ = torch::jit::load(model_path);
        module_.eval();

        // 2. Load Normalization Stats
        std::ifstream f_norm(norm_path);
        if (!f_norm.is_open()) {
            std::cerr << "[P2] Failed to open norm file: " << norm_path << std::endl;
            return false;
        }
        json j_norm;
        f_norm >> j_norm;
        
        mean_ = j_norm["mean"].get<std::vector<float>>();
        std_ = j_norm["std"].get<std::vector<float>>();

        // 3. Load Meta Config (v_min, v_max)
        std::ifstream f_meta(meta_path);
        if (f_meta.is_open()) {
            json j_meta;
            f_meta >> j_meta;
            if (j_meta.contains("v_min")) v_min_ = j_meta["v_min"];
            if (j_meta.contains("v_max")) v_max_ = j_meta["v_max"];
        }

        loaded_ = true;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[P2] Error loading SpeedCorrectorModel: " << e.what() << std::endl;
        return false;
    }
}

float SpeedCorrectorModel::inferVsafe(const std::vector<float>& features) {
    if (!loaded_ || features.size() != mean_.size()) {
        return -1.0f; // Error
    }

    // 1. Normalize
    std::vector<torch::jit::IValue> inputs;
    at::Tensor t = torch::zeros({1, (long)features.size()});
    auto accessor = t.accessor<float, 2>();
    
    for (size_t i = 0; i < features.size(); ++i) {
        float val = (features[i] - mean_[i]) / std_[i];
        accessor[0][i] = val;
    }
    inputs.push_back(t);

    // 2. Inference
    try {
        at::Tensor output = module_.forward(inputs).toTensor();
        float v_pred = output.item<float>();
        
        // 3. Post-process (Clamp)
        return std::clamp(v_pred, v_min_, v_max_);
        
    } catch (...) {
        return -1.0f;
    }
}

} // namespace pinn
