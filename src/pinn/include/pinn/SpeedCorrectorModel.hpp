#ifndef PINN_SPEED_CORRECTOR_MODEL_HPP
#define PINN_SPEED_CORRECTOR_MODEL_HPP

#include <torch/script.h>
#include <vector>
#include <string>

namespace pinn {

class SpeedCorrectorModel {
public:
    SpeedCorrectorModel();

    // Load model, norm stats, and meta config
    bool load(const std::string& model_path, 
              const std::string& norm_path,
              const std::string& meta_path);

    // Predict safe speed (m/s) given feature vector
    // Returns negative value if inference fails
    float inferVsafe(const std::vector<float>& features);

private:
    torch::jit::script::Module module_;
    
    // Normalization parameters
    std::vector<float> mean_;
    std::vector<float> std_;
    
    // Limits
    float v_min_ = 0.1f;
    float v_max_ = 2.0f;
    
    bool loaded_ = false;
};

} // namespace pinn

#endif
