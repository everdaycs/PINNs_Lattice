#ifndef PINN_EDGE_DYNAMICS_EVALUATOR_HPP
#define PINN_EDGE_DYNAMICS_EVALUATOR_HPP

#include <memory>
#include <mutex>
#include <unordered_map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pinn/FeasibilityModel.hpp"
#include "pinn/SpeedCorrectorModel.hpp"
#include "pinn/EdgeFeatureExtractor.hpp"

namespace pinn {

enum class Zone {
    GREEN,  // Safe, no change
    YELLOW, // Corrected speed
    RED     // Pruned
};

struct EvaluationResult {
    bool accepted = true;
    double extra_cost = 0.0;
    float v_safe = 1.0f; 
    Zone zone = Zone::GREEN;
};

struct EvaluatorStats {
    uint64_t total_queries = 0;
    uint64_t pruned_by_p1 = 0;
    uint64_t corrected_by_p2 = 0;
    uint64_t failed_p2_fallback = 0;
    uint64_t cache_hits = 0;
    
    void reset() {
        total_queries = pruned_by_p1 = corrected_by_p2 = failed_p2_fallback = cache_hits = 0;
    }
};

class EdgeDynamicsEvaluator {
public:
    EdgeDynamicsEvaluator();
    
    /**
     * @brief Path-based constructor for easier integration
     */
    EdgeDynamicsEvaluator(const std::string& p1_path, const std::string& p2_path, const std::string& lut_path);
    
    // 初始化与配置
    void configure(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node, const std::string& param_ns = "pinn");
    
    /**
     * @brief Simple interface for grid search
     */
    EvaluationResult evaluate(
        unsigned int prim_id,
        float v0,
        float mu
    );

    /**
     * @brief 核心评估函数
     * @param prim_id 原语类型的唯一ID (用于缓存Key)
     * @param prim_info 几何信息
     * @param v0 初始速度 (m/s)
     * @param mu 摩擦系数
     * @param veh 车辆参数
     */
    EvaluationResult evaluate(
        unsigned int prim_id,
        const PrimitiveInfo& prim_info,
        float v0,
        float mu,
        const VehicleParams& veh
    );

    /**
     * @brief Batch version for high-performance search
     */
    std::vector<EvaluationResult> evaluateBatch(
        const std::vector<unsigned int>& prim_ids,
        const std::vector<PrimitiveInfo>& prim_infos,
        float v0,
        float mu,
        const VehicleParams& veh
    );
    
    void logStats(const rclcpp::Logger& logger);

private:
    uint64_t getCacheKey(unsigned int prim_id, float v0, float mu);
    uint64_t checkCache(unsigned int prim_id, float v0, float mu, float& risk_out, float& v_safe_out);
    void updateCache(uint64_t key, float risk, float v_safe);

    // Models
    std::shared_ptr<FeasibilityModel> p1_;
    std::shared_ptr<SpeedCorrectorModel> p2_;
    
    // Params
    bool enable_p1_ = false;
    bool enable_p2_ = false;
    double t_ok_ = 0.2;
    double t_try_ = 0.8;
    double w_risk_ = 5.0;
    double w_time_ = 1.0;
    double w_fail_ = 100.0;
    std::string fallback_mode_ = "prune";
    float bin_v0_ = 0.1f;
    float bin_mu_ = 0.1f;
    
    // Cache
    struct CacheValue {
        float risk;
        float v_safe;
    };
    // Key: Combined hash of ID, quantized v0, quantized mu
    std::unordered_map<uint64_t, CacheValue> cache_;
    std::mutex cache_mutex_;
    
    // Stats
    EvaluatorStats stats_;
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
};

} // namespace pinn

#endif
