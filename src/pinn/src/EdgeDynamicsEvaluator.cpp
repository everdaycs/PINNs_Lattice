#include "pinn/EdgeDynamicsEvaluator.hpp"
#include <cmath>
#include <iostream>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace pinn {

EdgeDynamicsEvaluator::EdgeDynamicsEvaluator() 
    : p1_(std::make_shared<FeasibilityModel>()),
      p2_(std::make_shared<SpeedCorrectorModel>()) {
}

EdgeDynamicsEvaluator::EdgeDynamicsEvaluator(const std::string& p1_path, const std::string& p2_path, const std::string& lut_path)
    : EdgeDynamicsEvaluator() {
    enable_p1_ = !p1_path.empty();
    enable_p2_ = !p2_path.empty();
    
    // Default norm files using same naming convention if not provided
    if (enable_p1_) {
       std::string norm = p1_path;
       size_t last_dot = p1_path.find_last_of(".");
       if (last_dot != std::string::npos) {
           norm = p1_path.substr(0, last_dot) + ".json";
       }
       p1_->load(p1_path, norm);
    }
    if (enable_p2_) {
       std::string norm = p2_path;
       size_t last_dot = p2_path.find_last_of(".");
       if (last_dot != std::string::npos) {
           norm = p2_path.substr(0, last_dot) + ".json";
       }
       p2_->load(p2_path, norm, lut_path);
    }
}

void EdgeDynamicsEvaluator::configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& ns) {
    node_ = node;
    // 1. Basic Switches
    if (!node->has_parameter(ns + ".enable_p1")) node->declare_parameter(ns + ".enable_p1", rclcpp::ParameterValue(false));
    if (!node->has_parameter(ns + ".enable_p2")) node->declare_parameter(ns + ".enable_p2", rclcpp::ParameterValue(false));
    
    node->get_parameter(ns + ".enable_p1", enable_p1_);
    node->get_parameter(ns + ".enable_p2", enable_p2_);
    
    // 2. Thresholds & Weights
    if (!node->has_parameter(ns + ".threshold_ok")) node->declare_parameter(ns + ".threshold_ok", rclcpp::ParameterValue(0.2));
    if (!node->has_parameter(ns + ".threshold_try")) node->declare_parameter(ns + ".threshold_try", rclcpp::ParameterValue(0.8));
    if (!node->has_parameter(ns + ".w_risk")) node->declare_parameter(ns + ".w_risk", rclcpp::ParameterValue(5.0));
    if (!node->has_parameter(ns + ".w_time")) node->declare_parameter(ns + ".w_time", rclcpp::ParameterValue(1.0));
    if (!node->has_parameter(ns + ".w_fail")) node->declare_parameter(ns + ".w_fail", rclcpp::ParameterValue(100.0));
    if (!node->has_parameter(ns + ".fallback_mode")) node->declare_parameter(ns + ".fallback_mode", rclcpp::ParameterValue(std::string("prune")));
    if (!node->has_parameter(ns + ".v0_bin_size")) node->declare_parameter(ns + ".v0_bin_size", rclcpp::ParameterValue(0.1f));
    if (!node->has_parameter(ns + ".mu_bin_size")) node->declare_parameter(ns + ".mu_bin_size", rclcpp::ParameterValue(0.1f));

    node->get_parameter(ns + ".threshold_ok", t_ok_);
    node->get_parameter(ns + ".threshold_try", t_try_);
    node->get_parameter(ns + ".w_risk", w_risk_);
    node->get_parameter(ns + ".w_time", w_time_);
    node->get_parameter(ns + ".w_fail", w_fail_);
    node->get_parameter(ns + ".fallback_mode", fallback_mode_);
    node->get_parameter(ns + ".v0_bin_size", bin_v0_);
    node->get_parameter(ns + ".mu_bin_size", bin_mu_);

    // 3. Load Models
    std::string p1_model, p1_norm;
    std::string p2_model, p2_norm, p2_meta;
    
    if (enable_p1_) {
        if (!node->has_parameter(ns + ".p1_model_path")) node->declare_parameter(ns + ".p1_model_path", rclcpp::ParameterValue(""));
        if (!node->has_parameter(ns + ".p1_norm_path")) node->declare_parameter(ns + ".p1_norm_path", rclcpp::ParameterValue(""));
        node->get_parameter(ns + ".p1_model_path", p1_model);
        node->get_parameter(ns + ".p1_norm_path", p1_norm);
        if (!p1_->load(p1_model, p1_norm)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to load P1 model. Disabling P1.");
            enable_p1_ = false;
        } else {
            RCLCPP_INFO(node->get_logger(), "P1 Model Loaded.");
        }
    }
    
    if (enable_p2_) {
        if (!node->has_parameter(ns + ".p2_model_path")) node->declare_parameter(ns + ".p2_model_path", rclcpp::ParameterValue(""));
        if (!node->has_parameter(ns + ".p2_norm_path")) node->declare_parameter(ns + ".p2_norm_path", rclcpp::ParameterValue(""));
        if (!node->has_parameter(ns + ".p2_meta_path")) node->declare_parameter(ns + ".p2_meta_path", rclcpp::ParameterValue(""));
        node->get_parameter(ns + ".p2_model_path", p2_model);
        node->get_parameter(ns + ".p2_norm_path", p2_norm);
        node->get_parameter(ns + ".p2_meta_path", p2_meta);
        if (!p2_->load(p2_model, p2_norm, p2_meta)) {
            RCLCPP_ERROR(node->get_logger(), "Failed to load P2 model. Disabling P2.");
            enable_p2_ = false;
        } else {
            RCLCPP_INFO(node->get_logger(), "P2 Model Loaded.");
        }
    }
}

uint64_t EdgeDynamicsEvaluator::getCacheKey(unsigned int prim_id, float v0, float mu) {
    int v_bin = static_cast<int>(v0 / bin_v0_);
    int mu_bin = static_cast<int>(mu / bin_mu_);
    return (static_cast<uint64_t>(prim_id) << 32) | (static_cast<uint64_t>(v_bin) << 16) | static_cast<uint64_t>(mu_bin);
}

uint64_t EdgeDynamicsEvaluator::checkCache(unsigned int prim_id, float v0, float mu, float& risk_out, float& v_safe_out) {
    uint64_t key = getCacheKey(prim_id, v0, mu);
    std::lock_guard<std::mutex> lock(cache_mutex_);
    auto it = cache_.find(key);
    if (it != cache_.end()) {
        risk_out = it->second.risk;
        v_safe_out = it->second.v_safe;
        return 1; // hit
    }
    return 0; // miss
}

void EdgeDynamicsEvaluator::updateCache(uint64_t key, float risk, float v_safe) {
    std::lock_guard<std::mutex> lock(cache_mutex_);
    cache_[key] = {risk, v_safe};
}

std::vector<EvaluationResult> EdgeDynamicsEvaluator::evaluateBatch(
    const std::vector<unsigned int>& prim_ids,
    const std::vector<PrimitiveInfo>& prim_infos,
    float v0,
    float mu,
    const VehicleParams& veh) 
{
    size_t n = prim_ids.size();
    std::vector<EvaluationResult> results(n);
    std::vector<size_t> missing_indices;
    std::vector<uint64_t> missing_keys;
    
    // 1. Check Cache
    for (size_t i = 0; i < n; ++i) {
        float risk, v_safe;
        if (checkCache(prim_ids[i], v0, mu, risk, v_safe)) {
            stats_.total_queries++;
            stats_.cache_hits++;
            
            // Strategy logic
            results[i].v_safe = v_safe;
            if (risk <= t_ok_) {
                results[i].accepted = true;
                results[i].zone = Zone::GREEN;
                results[i].extra_cost = w_risk_ * risk;
            } else if (risk <= t_try_) {
                results[i].accepted = true;
                results[i].zone = Zone::YELLOW;
                results[i].v_safe = v_safe;
                double scale = (v0 > 0.01) ? (v0 / v_safe) : 1.0;
                results[i].extra_cost = w_time_ * (prim_infos[i].length * scale) + w_risk_ * risk;
            } else {
                results[i].accepted = false;
                results[i].zone = Zone::RED;
                results[i].extra_cost = w_fail_;
            }
        } else {
            missing_indices.push_back(i);
            missing_keys.push_back(getCacheKey(prim_ids[i], v0, mu));
        }
    }
    
    if (missing_indices.empty()) return results;
    
    // 2. Prepare Batch Inference
    std::vector<float> all_features;
    for (size_t idx : missing_indices) {
        auto feat = EdgeFeatureExtractor::extract(prim_infos[idx], v0, mu, veh);
        all_features.insert(all_features.end(), feat.begin(), feat.end());
    }
    
    std::vector<float> risks(missing_indices.size(), 0.0f);
    if (enable_p1_) {
        risks = p1_->inferBatch(all_features, missing_indices.size());
    }
    
    std::vector<float> vsafes(missing_indices.size(), v0);
    if (enable_p2_) {
        vsafes = p2_->inferVsafeBatch(all_features, missing_indices.size());
    }
    
    // 3. Process Results and Update Cache
    for (size_t j = 0; j < missing_indices.size(); ++j) {
        size_t i = missing_indices[j];
        float risk = risks[j];
        float v_safe = vsafes[j];
        
        updateCache(missing_keys[j], risk, v_safe);
        stats_.total_queries++;
        
        results[i].v_safe = v_safe;
        if (risk <= t_ok_) {
            results[i].accepted = true;
            results[i].zone = Zone::GREEN;
            results[i].extra_cost = w_risk_ * risk;
        } else if (risk <= t_try_) {
            results[i].accepted = true;
            results[i].zone = Zone::YELLOW;
            results[i].v_safe = v_safe;
            double scale = (v0 > 0.01) ? (v0 / v_safe) : 1.0;
            results[i].extra_cost = w_time_ * (prim_infos[i].length * scale) + w_risk_ * risk;
        } else {
            results[i].accepted = false;
            results[i].zone = Zone::RED;
            results[i].extra_cost = w_fail_;
        }
    }
    
    return results;
}

EvaluationResult EdgeDynamicsEvaluator::evaluate(unsigned int prim_id, float v0, float mu) {
    // Dummy PrimitiveInfo for grid search (approx distance = 1m)
    PrimitiveInfo info;
    info.length = 1.0f;
    info.max_curvature = 0.0f; 
    
    VehicleParams veh; // Defaults
    return evaluate(prim_id, info, v0, mu, veh);
}

EvaluationResult EdgeDynamicsEvaluator::evaluate(unsigned int prim_id, const PrimitiveInfo& info, float v0, float mu, const VehicleParams& veh) {
    std::lock_guard<std::mutex> lock(cache_mutex_); // For stats thread safety
    stats_.total_queries++;
    
    // Baseline: if disabled, accept everything
    if (!enable_p1_ && !enable_p2_) {
        return {true, 0.0, v0, Zone::GREEN};
    }

    EvaluationResult res;
    res.accepted = true;
    res.extra_cost = 0.0;
    res.v_safe = v0;
    res.zone = Zone::GREEN;
    
    float risk = 0.0f;
    float v_safe_pred = v0;
    
    // 1. Check Cache
    int v_bin = static_cast<int>(v0 / bin_v0_);
    int mu_bin = static_cast<int>(mu / bin_mu_);
    uint64_t key = (static_cast<uint64_t>(prim_id) << 32) | (static_cast<uint64_t>(v_bin) << 16) | static_cast<uint64_t>(mu_bin);
    
    auto it = cache_.find(key);
    if (it != cache_.end()) {
        risk = it->second.risk;
        v_safe_pred = it->second.v_safe;
        stats_.cache_hits++;
    } else {
        // inference
        auto feats = EdgeFeatureExtractor::extract(info, v0, mu, veh);
        
        if (enable_p1_) {
            risk = p1_->infer(feats);
        }
        
        if (enable_p2_ && risk > t_ok_ && risk <= t_try_) {
            v_safe_pred = p2_->inferVsafe(feats);
        }
        
        // Update Cache
        cache_[key] = {risk, v_safe_pred};
    }
    
    // --- Strategy Evaluation ---
    res.v_safe = v_safe_pred;
    
    // 1. Safe Zone
    if (risk <= t_ok_) {
        res.accepted = true;
        res.zone = Zone::GREEN;
        res.extra_cost = w_risk_ * risk;
    } else if (risk <= t_try_) {
        // 2. Try Zone (Correction)
        if (enable_p2_) {
            if (v_safe_pred < 0.1f) {
                 stats_.failed_p2_fallback++;
                 res.accepted = (fallback_mode_ != "prune");
                 res.zone = Zone::RED;
                 res.extra_cost = w_fail_;
            } else {
                stats_.corrected_by_p2++;
                res.accepted = true;
                res.zone = Zone::YELLOW;
                res.v_safe = v_safe_pred;
                
                double scale = (v0 > 0.01) ? (v0 / v_safe_pred) : 1.0;
                res.extra_cost = w_time_ * (info.length * scale) + w_risk_ * risk;
            }
        } else {
             res.accepted = true;
             res.zone = Zone::GREEN;
             res.extra_cost = w_risk_ * risk; 
        }
    } else {
        // 3. Prune Zone
        stats_.pruned_by_p1++;
        res.accepted = false;
        res.zone = Zone::RED;
    }
    
    return res;
}

void EdgeDynamicsEvaluator::logStats(const rclcpp::Logger& logger) {
    std::lock_guard<std::mutex> lock(cache_mutex_); // Locking just in case
    if (stats_.total_queries > 0 && node_ != nullptr) {
        RCLCPP_INFO_THROTTLE(logger, *node_->get_clock(), 5000, 
            "[PINN Stats] Total: %lu | Pruned: %lu | Corrected: %lu | CacheHit: %.2f%%",
            stats_.total_queries, stats_.pruned_by_p1, stats_.corrected_by_p2, 
            (double)stats_.cache_hits / (stats_.total_queries + 1e-6) * 100.0);
    }
}

} // namespace pinn
