import os
import pandas as pd
import numpy as np

class DatasetWriter:
    def __init__(self, output_dir, file_prefix='shard'):
        self.output_dir = output_dir
        self.file_prefix = file_prefix
        
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)
            
        self.current_shard_idx = 0
        self._set_filename()
        
    def _set_filename(self):
        # find next available shard index to support resume
        while True:
            self.filename = os.path.join(self.output_dir, f"{self.file_prefix}_{self.current_shard_idx:03d}.parquet")
            if not os.path.exists(self.filename):
                break
            self.current_shard_idx += 1
            
    def write_batch(self, batch_data):
        """
        Args:
            batch_data: list of dicts or DataFrame
        """
        if not batch_data:
            return
            
        df = pd.DataFrame(batch_data)
        
        # Ensure efficient types
        # ...
        
        # Write to Parquet
        # If file exists? We chose a new name on init. But if we call write_batch multiple times...
        # Strategy: Each batch is a file? Or append?
        # Parquet is not easy to append unless using FastParquet or PyArrow tables.
        # Recommendation: Write one file per batch or accumulate in memory.
        # Given 2e5 samples, if batch=1000, we have 200 files. That's fine.
        # Or we can accumulate until some size.
        
        # For simplicity in this script: One shard per write call (managed by the caller to accumulate)
        # Caller calls write_batch when buffer is full.
        
        df.to_parquet(self.filename, compression='snappy')
        print(f"Saved {len(df)} samples to {self.filename}")
        self.current_shard_idx += 1
        self._set_filename() # Prepare for next batch
