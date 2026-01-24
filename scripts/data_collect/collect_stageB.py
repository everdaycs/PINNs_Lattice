import sys
import os
import argparse
import yaml
import time
import numpy as np
import multiprocessing

# Add src to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../src'))

from data_collect.data_collection.primitive_sampler import PrimitiveSampler
from data_collect.data_collection.rollout_sim import RolloutSim
from data_collect.data_collection.dataset_writer import DatasetWriter
from data_collect.data_collection.map_manager import MapManager
from data_collect.data_collection.feature_extractor import FeatureExtractor

def worker_wrapper(args):
    return worker_search_and_collect(*args)

def transform_traj(traj, x, y, yaw):
    """Transform local traj to global frame (x,y,yaw)"""
    # traj: [N, 3]
    # x, y, yaw: scalar
    
    c = np.cos(yaw)
    s = np.sin(yaw)
    
    # R * p + t
    # p = traj[:, :2]
    # R = [[c, -s], [s, c]]
    
    new_traj = np.zeros_like(traj)
    new_traj[:, 0] = x + c * traj[:, 0] - s * traj[:, 1]
    new_traj[:, 1] = y + s * traj[:, 0] + c * traj[:, 1]
    new_traj[:, 2] = traj[:, 2] + yaw
    
    return new_traj

def worker_search_and_collect(worker_id, config, num_samples, seed_offset):
    np.random.seed(config['seed'] + seed_offset + int(time.time()*1000)%1000 + worker_id)
    
    # Load Map (Each worker loads map to avoid shared memory issues or pickling large objects)
    map_manager = MapManager(config['map']['yaml_path'], config['map']['padding'])
    
    sampler = PrimitiveSampler(config)
    sim = RolloutSim(config)
    extractor = FeatureExtractor(map_manager, config)
    
    samples = []
    
    mu_range = config['rollout']['mu_range']
    v0_range = config['rollout']['v0_range']
    
    collected_count = 0
    max_expansions = config['search']['max_expansions']
    
    while collected_count < num_samples:
        # 1. Random Search State (Mimic A* expansion)
        # Pick a valid free point in map
        # Retry loop
        for _ in range(100):
            rx = np.random.uniform(map_manager.x_min, map_manager.x_max)
            ry = np.random.uniform(map_manager.y_min, map_manager.y_max)
            if not map_manager.check_collision(rx, ry):
                break
        
        start_x, start_y = rx, ry
        start_yaw = np.random.uniform(-np.pi, np.pi)
        
        # 2. Expand Primitives
        # Generate N primitives from this state
        # In Smac, typically ~5-7 primitives
        # Here we sample K primitives using our Sampler
        
        num_prims = 5
        for _ in range(num_prims):
            local_traj, meta = sampler.sample()
            
            # Transform to Global
            global_traj = transform_traj(local_traj, start_x, start_y, start_yaw)
            
            # 3. Environment Features (Pre-check)
            # If start is valid, check trajectory environment
            env_feats = extractor.extract_env_features(global_traj)
            
            # Downsampling Strategy (Hard Mining)
            # If env is too safe (cost=0), maybe skip with high prob?
            p_accept = config['search']['prob_sample']
            if config['search']['hard_mining']:
                if env_feats['env_cost_max'] > 0 or env_feats['env_clearance_min'] < 0.5:
                    p_accept = 1.0 # Always keep risky ones
            
            if np.random.random() > p_accept:
                continue
                
            # 4. Rollout (Labeling)
            v0 = np.random.uniform(v0_range[0], v0_range[1])
            mu = np.random.uniform(mu_range[0], mu_range[1])
            
            res = sim.run_rollout(global_traj, v0, mu, map_manager)
            
            # 5. Save
            sample = {}
            # ... copy meta ...
            sample.update(meta)
            sample.update(env_feats)
            sample['v0'] = v0
            sample['mu'] = mu
            sample['collision'] = res['collision']
            sample['feasible'] = res['feasible']
            # ... Copy result stats ...
            sample['risk'] = res['risk']
            sample['max_ay'] = res['max_ay']
            sample['max_error'] = res['max_error']
            
            samples.append(sample)
            collected_count += 1
            
            if collected_count >= num_samples:
                break
                
    return samples

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', type=str, default='configs/data_collect_stageB.yaml')
    args = parser.parse_args()
    
    # Path handling
    config_path = os.path.abspath(args.config)
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        
    out_dir = os.path.abspath(config['output_dir'])
    writer = DatasetWriter(out_dir, file_prefix='shard_stageB')
    
    num_workers = config['num_workers']
    total_samples = config['num_samples']
    batch_size = config['batch_size']
    
    pool = multiprocessing.Pool(num_workers)
    
    tasks = []
    # Similar chunk logic as Stage A
    num_batches = (total_samples + batch_size - 1) // batch_size
    for b in range(num_batches):
        current_batch = min(batch_size, total_samples - b*batch_size)
        tasks.append((b % num_workers, config, current_batch, b*100))
        
    print(f"Stage B Collection: Target={total_samples}")
    
    count = 0
    t0 = time.time()
    for batch_res in pool.imap_unordered(worker_wrapper, tasks):
        writer.write_batch(batch_res)
        count += len(batch_res)
        print(f"Progress: {count}/{total_samples}")
        
    pool.close()
    pool.join()
    print("Done Stage B")

if __name__ == '__main__':
    main()
