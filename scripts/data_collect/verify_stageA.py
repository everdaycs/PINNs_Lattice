import pandas as pd
import glob
import os

def verify(data_dir):
    files = glob.glob(os.path.join(data_dir, "*.parquet"))
    if not files:
        print("No data found")
        return
        
    df = pd.concat([pd.read_parquet(f) for f in files])
    print(f"Total samples: {len(df)}")
    print("\n--- Feasibility ---")
    print(df['feasible'].value_counts(normalize=True))
    
    print("\n--- Risk Stats ---")
    print(df['risk'].describe())
    
    print("\n--- Max Lat Accel ---")
    print(df['max_ay'].describe())
    
    print("\n--- Tracking Error ---")
    print(df['max_error'].describe())
    
if __name__ == "__main__":
    import sys
    d = sys.argv[1] if len(sys.argv) > 1 else "data/pinn_lattice_dataset/stageA"
    verify(d)
