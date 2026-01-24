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

class P1Dataset(Dataset):
    def __init__(self, data_path, schema_path):
        with open(schema_path, 'r') as f:
            self.schema = yaml.safe_load(f)
            
        self.feature_names = [f['name'] for f in self.schema['features']]
        self.target_name = self.schema['target']
        
        # Load Data (Parquet)
        files = glob.glob(os.path.join(data_path, "*.parquet"))
        if not files:
            raise ValueError(f"No parquet files found in {data_path}")
            
        df_list = []
        for f in files:
            df_list.append(pd.read_parquet(f))
        self.df = pd.concat(df_list, ignore_index=True)
        
        # Preprocessing: Map dataset columns to schema names if needed
        # Assuming collect script used specific names.
        # Mapping: 
        # length -> length_m
        # delta_theta -> delta_yaw_rad
        # delta_x -> delta_x_local_m
        # delta_y -> delta_y_local_m
        # mean_kappa -> curvature_mean_inv_m
        # max_kappa -> curvature_max_inv_m
        # v0 -> v0_mps
        # mu -> mu
        # wheelbase -> wheelbase_m
        # delta_max -> steering_limit_rad
        
        column_map = {
            'length': 'length_m',
            'delta_theta': 'delta_yaw_rad',
            'delta_x': 'delta_x_local_m',
            'delta_y': 'delta_y_local_m',
            'mean_kappa': 'curvature_mean_inv_m',
            'max_kappa': 'curvature_max_inv_m',
            'v0': 'v0_mps',
            'wheelbase': 'wheelbase_m',
            'delta_max': 'steering_limit_rad'
        }
        self.df.rename(columns=column_map, inplace=True)
        
        # Fill Missing
        for feat in self.schema['features']:
            name = feat['name']
            if name not in self.df.columns:
                print(f"Warning: Feature {name} missing in data, filling with {feat['missing']}")
                self.df[name] = feat['missing']
            else:
                self.df[name] = self.df[name].fillna(feat['missing'])
                
        # Calculate Norm Stats
        self.stats = {}
        for feat in self.schema['features']:
            name = feat['name']
            if feat.get('norm', True):
                mean = self.df[name].mean()
                std = self.df[name].std()
                if std < 1e-6: std = 1.0
                self.stats[name] = {'mean': float(mean), 'std': float(std)}
                # Apply normalization in memory for training
                self.df[name] = (self.df[name] - mean) / std
            else:
                self.stats[name] = {'mean': 0.0, 'std': 1.0}
        
        self.X = self.df[self.feature_names].values.astype(np.float32)
        self.y = self.df[self.target_name].values.astype(np.float32).reshape(-1, 1)

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx):
        return self.X[idx], self.y[idx]

    def save_artifacts(self, output_dir):
        # Save Norm Params
        norm_path = os.path.join(output_dir, 'p1_norm.json')
        with open(norm_path, 'w') as f:
            # Order matters! C++ will iterate vector.
            ordered_stats = []
            for feat in self.schema['features']:
                name = feat['name']
                s = self.stats[name]
                ordered_stats.append({
                    'name': name,
                    'mean': s['mean'],
                    'std': s['std']
                })
            json.dump({'features': ordered_stats}, f, indent=2)

class P1Model(nn.Module):
    def __init__(self, input_dim):
        super(P1Model, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(input_dim, 128),
            nn.ReLU(),
            nn.Dropout(0.1),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 1) # Regression: Risk score
        )
        
    def forward(self, x):
        return self.net(x)

def train():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, required=True, help="generated parquet dir")
    parser.add_argument('--schema', type=str, default='configs/pinn/feature_schema.yaml')
    parser.add_argument('--output', type=str, default='artifacts/pinn')
    parser.add_argument('--epochs', type=int, default=50)
    args = parser.parse_args()
    
    os.makedirs(args.output, exist_ok=True)
    
    dataset = P1Dataset(args.data, args.schema)
    train_size = int(0.8 * len(dataset))
    val_size = len(dataset) - train_size
    train_ds, val_ds = torch.utils.data.random_split(dataset, [train_size, val_size])
    
    train_loader = DataLoader(train_ds, batch_size=256, shuffle=True)
    val_loader = DataLoader(val_ds, batch_size=256)
    
    model = P1Model(len(dataset.feature_names))
    optimizer = optim.Adam(model.parameters(), lr=1e-3)
    criterion = nn.MSELoss() # Regress risk
    
    best_val_loss = float('inf')
    
    for epoch in range(args.epochs):
        model.train()
        train_loss = 0.0
        for X, y in train_loader:
            optimizer.zero_grad()
            pred = model(X)
            loss = criterion(pred, y)
            loss.backward()
            optimizer.step()
            train_loss += loss.item()
            
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for X, y in val_loader:
                pred = model(X)
                loss = criterion(pred, y)
                val_loss += loss.item()
                
        avg_val_loss = val_loss / len(val_loader)
        print(f"Epoch {epoch}: Train Loss {train_loss/len(train_loader):.4f}, Val Loss {avg_val_loss:.4f}")
        
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            # Save TorchScript
            example_input = torch.zeros(1, len(dataset.feature_names))
            traced_script_module = torch.jit.trace(model, example_input)
            traced_script_module.save(os.path.join(args.output, "p1_model.ts"))
            
    # Save Norms
    dataset.save_artifacts(args.output)
    print("Training Done. Artifacts saved.")

if __name__ == '__main__':
    train()
