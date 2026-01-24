#ifndef PINN_FEASIBILITY_MODEL_HPP
#define PINN_FEASIBILITY_MODEL_HPP

#include <vector>
#include <string>
#include <memory>
#include <unordered_map>
#include <torch/script.h> // LibTorch

namespace pinn {

struct FeatureNorm {
    std::string name;
    float mean;
    float std;
};

class FeasibilityModel {
public:
    FeasibilityModel();
    bool load(const std::string& model_path, const std::string& norm_path);
    
    // Core Inference
    // Returns risk score (higher is worse)
    float inferRisk(const std::vector<float>& features);
    
    // Feature helper provided by separate class, but model holds norm params
    const std::vector<FeatureNorm>& getNormParams() const { return norms_; }

    float infer(const std::vector<float>& features);

private:
    torch::jit::script::Module module_;
    std::vector<FeatureNorm> norms_;
    bool loaded_ = false;
    
    // Caching (optional advanced feature)
    // std::unordered_map<std::string, float> cache_;
};

} // namespace pinn

#endif // PINN_FEASIBILITY_MODEL_HPP
