#!/usr/bin/env python3

import os
import re
import glob

def remove_plugins_from_sdf(file_path):
    """Remove all plugin blocks from an SDF file"""
    with open(file_path, 'r') as f:
        content = f.read()
    
    # Remove plugin blocks using regex
    # This pattern matches from <plugin> to its closing </plugin>
    plugin_pattern = r'\s*<!--[^>]*-->\s*<plugin[^>]*>.*?</plugin>\s*'
    content = re.sub(plugin_pattern, '', content, flags=re.DOTALL)
    
    # Also remove just plugin blocks without comments
    plugin_pattern2 = r'\s*<plugin[^>]*>.*?</plugin>\s*'
    content = re.sub(plugin_pattern2, '', content, flags=re.DOTALL)
    
    # Clean up any extra blank lines
    content = re.sub(r'\n\s*\n\s*\n', '\n\n', content)
    
    # Also fix gravity settings that were disabled for plugins
    content = re.sub(r'<gravity>false</gravity>', '<gravity>true</gravity>', content)
    
    with open(file_path, 'w') as f:
        f.write(content)
    
    print(f"Cleaned plugins from: {file_path}")

def main():
    # Find all model.sdf files in the models directory
    model_files = glob.glob('/home/navid/vrx_ws/src/plastiSim/models/**/model.sdf', recursive=True)
    
    for model_file in model_files:
        remove_plugins_from_sdf(model_file)
    
    print(f"\nProcessed {len(model_files)} model files.")
    print("All plugins have been removed from plastiSim models.")

if __name__ == "__main__":
    main()
