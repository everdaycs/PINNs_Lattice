import torch
import torch.nn as nn
import torch.optim as optim
import pandas as pd
import numpy as np
import yaml
import json
import argparse
import os
import glob
from torch.utils.data import Dataset, DataLoader

class P2Dataset(Dataset):
    def __init__(self, data_path, schema_path, v_min=0.1, v_max=2.0, g=9.81):
        with open(schema_path, 'r') as f:
            self.schema = yaml.safe_load(f)
            
        self.feature_names = [f['name'] for f in self.schema['features']]
        # Generate indices for critical features needed for physics rule
        self.idx_map = {name: i for i, name in enumerate(self.feature_names)}
        
        # Load Data
        files = glob.glob(os.path.join(data_path, "*.parquet"))
        if not files:
            raise ValueError(f"No parquet files found in {data_path}")
            
        df_list = []
        for f in files:
            df_list.append(pd.read_parquet(f))
        self.df = pd.concat(df_list, ignore_index=True)
        
        # Column Mapping (Reuse P1 logic + add robustness)
        feature_cols = []
        # Fallback map based on collect_stageA/B.py naming
        col_map = {
            'length_m': 'length',
            'delta_yaw_rad': 'delta_theta',
            'delta_x_local_m': 'delta_x',
            'delta_y_local_m': 'delta_y',
            'curvature_mean_inv_m': 'mean_kappa',
            'curvature_max_inv_m': 'max_kappa',
            'v0_mps': 'v0',
            'mu': 'mu',
            'wheelbase_m': 'wheelbase',
            'steering_limit_rad': 'delta_max'
        }
        
        # Prepare inputs X
        self.X_raw = np.zeros((len(self.df), len(self.feature_names)), dtype=np.float32)
        for i, feat_name in enumerate(self.feature_names):
            col = col_map.get(feat_name, feat_name) # Try map, else use exact name
            if col in self.df.columns:
                self.X_raw[:, i] = self.df[col].values
            else:
                # Use default from schema if missing
                default = next(f['missing'] for f in self.schema['features'] if f['name'] == feat_name)
                self.X_raw[:, i] = default

        # Process labels Y: Generate Pseudo-Label v_safe
        # Formula: v = sqrt(mu * g / max_kappa)
        mu_col = self.idx_map.get('mu', -1)
        kappa_col = self.idx_map.get('curvature_max_inv_m', -1)
        
        if mu_col != -1 and kappa_col != -1:
            mus = self.X_raw[:, mu_col]
            kappas = np.abs(self.X_raw[:, kappa_col])
            # Avoid division by zero
            kappas = np.maximum(kappas, 1e-4)
            
            v_physics = np.sqrt(mus * g / kappas)
            
            # Clip to [v_min, v_max]
            self.Y = np.clip(v_physics, v_min, v_max).astype(np.float32)
        else:
            print("Warning: Could not compute physics label, checking for 'v_safe' column...")
            if 'v_safe' in self.df.columns:
                self.Y = self.df['v_safe'].values.astype(np.float32)
            else:
                raise ValueError("Neither required columns for physics (mu, max_kappa) nor 'v_safe' label found.")

        # Calc Normalization Stats
        self.mean = np.mean(self.X_raw, axis=0)
        self.std = np.std(self.X_raw, axis=0)
        self.std[self.std < 1e-6] = 1.0 # Avoid div/0
        
        # Normalized X
        self.X = (self.X_raw - self.mean) / self.std

    def __len__(self):
        return len(self.X)

    def __getitem__(self, idx):
        return torch.tensor(self.X[idx]), torch.tensor(self.Y[idx]).unsqueeze(0)
        
    def save_artifacts(self, output_dir):
        # Save Norms
        norm_data = {
            "mean": self.mean.tolist(),
            "std": self.std.tolist(),
            "features": self.feature_names
        }
        with open(os.path.join(output_dir, "p2_norm.json"), 'w') as f:
            json.dump(norm_data, f, indent=2)

class P2Model(nn.Module):
    def __init__(self, input_dim):
        super(P2Model, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 64),
            nn.ReLU(),
            nn.Linear(64, 1),
            nn.Sigmoid() # Output 0-1, will scale later? Or just Linear?
            # Physics Labels are in [v_min, v_max], e.g., [0.1, 2.0].
            # Let's use ReLU output or just Linear.
            # Sigmoid is good if we predict ratio of v_max.
            # For simplicity, let's use Linear and rely on Loss to guide.
        )
        # Replacing last layer with simple Linear for regression
        self.net[4] = nn.Linear(64, 1) # No activation at end for pure regression

    def forward(self, x):
        return self.net(x)

def train():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, required=True)
    parser.add_argument('--schema', type=str, default='configs/pinn/feature_schema.yaml')
    parser.add_argument('--output', type=str, default='artifacts/pinn')
    parser.add_argument('--epochs', type=int, default=50)
    args = parser.parse_args()
    
    os.makedirs(args.output, exist_ok=True)
    
    # meta config
    v_min, v_max = 0.1, 2.0
    
    try:
        dataset = P2Dataset(args.data, args.schema, v_min=v_min, v_max=v_max)
    except Exception as e:
        print(f"Dataset creation failed: {e}")
        return

    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_ds, val_ds = torch.utils.data.random_split(dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_ds, batch_size=256, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=256)
    
    model = P2Model(len(dataset.feature_names))
    optimizer = optim.Adam(model.parameters(), lr=1e-3)
    criterion = nn.MSELoss()
    
    best_loss = float('inf')
    
    for epoch in range(args.epochs):
        model.train()
        total_loss = 0
        for X, y in train_loader:
            optimizer.zero_grad()
            pred = model(X)
            loss = criterion(pred, y)
            loss.backward()
            optimizer.step()
            total_loss += loss.item()
            
        if epoch % 5 == 0:
            print(f"Epoch {epoch} Loss: {total_loss/len(train_loader):.4f}")
            
    # Save Model
    model.eval()
    example_input = torch.zeros(1, len(dataset.feature_names))
    traced = torch.jit.trace(model, example_input)
    traced.save(os.path.join(args.output, "p2_model.ts"))
    
    dataset.save_artifacts(args.output)
    
    # Save Meta
    meta = {
        "v_min": v_min,
        "v_max": v_max,
        "description": "Output is predicted v_safe in m/s directly."
    }
    with open(os.path.join(args.output, "p2_meta.json"), 'w') as f:
        json.dump(meta, f, indent=2)
        
    print("P2 Training Complete.")

if __name__ == '__main__':
    train()
