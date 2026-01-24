import sys
import os
import argparse
import yaml
import time
import numpy as np
import multiprocessing
from datetime import datetime

# Add src to sys.path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../src'))

from data_collect.data_collection.primitive_sampler import PrimitiveSampler
from data_collect.data_collection.rollout_sim import RolloutSim
from data_collect.data_collection.dataset_writer import DatasetWriter

def worker_wrapper(args):
    return worker_generate_batch(*args)

def worker_generate_batch(worker_id, config, num_samples, seed_offset):
    """
    Worker process to generate a batch of samples
    """
    np.random.seed(config['seed'] + seed_offset + int(time.time()*1000)%1000 + worker_id)
    
    sampler = PrimitiveSampler(config)
    sim = RolloutSim(config)
    
    samples = []
    
    mu_range = config['rollout']['mu_range']
    v0_range = config['rollout']['v0_range']
    
    for i in range(num_samples):
        # 1. Sample Geometry
        traj, meta = sampler.sample()
        
        # 2. Sample Dynamic Conditions
        v0 = np.random.uniform(v0_range[0], v0_range[1])
        mu = np.random.uniform(mu_range[0], mu_range[1])
        
        # 3. Rollout
        res = sim.run_rollout(traj, v0, mu)
        
        # 4. Flatten Data
        
        sample = {}
        # Meta
        sample['primitive_type'] = meta['primitive_type']
        sample['length'] = meta['length']
        sample['mean_kappa'] = meta['mean_kappa']
        sample['max_kappa'] = meta['max_kappa']
        sample['delta_x'] = meta['delta_x']
        sample['delta_y'] = meta['delta_y']
        sample['delta_theta'] = meta['delta_theta']
        
        # Dynamics X
        sample['v0'] = v0
        sample['mu'] = mu
        
        # Vehicle Utils (Static for this dataset but good to record)
        sample['wheelbase'] = config['vehicle']['wheelbase']
        sample['delta_max'] = config['vehicle']['max_steering_angle']
        sample['R_min'] = config['primitive']['R_min']
        
        # Labels Y
        sample['feasible'] = res['feasible']
        sample['risk'] = res['risk']
        sample['max_ay'] = res['max_ay']
        sample['max_delta'] = res['max_delta']
        sample['max_error'] = res['max_error']
        sample['rollout_success'] = res['rollout_success']
        
        sample['seed'] = config['seed']
        sample['timestamp'] = time.time()
        
        samples.append(sample)
        
    return samples

def main():
    parser = argparse.ArgumentParser(description="Stage A Data Collection")
    parser.add_argument('--config', type=str, default='configs/data_collect_stageA.yaml')
    args = parser.parse_args()
    
    # Load Config
    # handle relative path
    config_path = args.config
    if not os.path.isabs(config_path):
        config_path = os.path.join(os.getcwd(), config_path)

    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
        
    print(f"Loaded config from {config_path}")
    
    total_samples = config['num_samples']
    num_workers = config['num_workers']
    batch_size = config['batch_size'] # Samples per save file
    
    # 修正：绝对路径
    out_dir = config['output_dir']
    if not os.path.isabs(out_dir):
        out_dir = os.path.join(os.getcwd(), out_dir)
        
    writer = DatasetWriter(out_dir)
    
    pool = multiprocessing.Pool(processes=num_workers)
    
    print(f"Starting data collection: Target={total_samples}, Workers={num_workers}, Batch={batch_size}")
    
    tasks = []
    num_batches = (total_samples + batch_size - 1) // batch_size
    
    for b in range(num_batches):
        current_batch_size = min(batch_size, total_samples - b * batch_size)
        tasks.append((b % num_workers, config, current_batch_size, b * 997))

    samples_collected = 0
    t0 = time.time()

    # Use imap_unordered to handle results as they come in
    for batch_results in pool.imap_unordered(worker_wrapper, tasks):
        writer.write_batch(batch_results)
        samples_collected += len(batch_results)
        elapsed = time.time() - t0
        rate = samples_collected / elapsed if elapsed > 0 else 0
        print(f"Progress: {samples_collected}/{total_samples} ({samples_collected/total_samples*100:.1f}%) | Rate: {rate:.1f} samples/s")

    pool.close()
    pool.join()
    print("Done!")

if __name__ == '__main__':
    main()
