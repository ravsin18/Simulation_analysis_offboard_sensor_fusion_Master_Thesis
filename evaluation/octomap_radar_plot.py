#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import glob

# === USER INPUT ===
input_folder = input("Enter path to evaluation folder (e.g., /home/singh/ros2_sensor_ws/evaluation/empty_room): ").strip()

if not os.path.isdir(input_folder):
    print(f"Error: '{input_folder}' is not a valid folder.")
    exit(1)

output_folder = os.path.join(input_folder, "radar_plots")
os.makedirs(output_folder, exist_ok=True)

csv_files = glob.glob(os.path.join(input_folder, "*.csv"))

# === METRIC GROUPS ===
voxel_metrics = ['Occupied Voxels', 'Free Voxels', 'Unknown Voxels']
tnfpfn_metrics = ['3D True Positives', '3D False Positives', '3D True Negatives', '3D False Negatives']
quality_metrics = ['3D Precision', '3D Recall', '3D F1-Score', '3D IoU', 'Completeness (%)']

# === LOAD DATA ===
data = {}
for file in csv_files:
    try:
        df = pd.read_csv(file)
        df.set_index('Metric', inplace=True)
        filename = os.path.basename(file).replace(".csv", "")
        # Multiply quality metrics by 100
        for metric in ['3D Precision', '3D Recall', '3D F1-Score', '3D IoU']:
            if metric in df.index:
                df.at[metric, 'Value'] *= 100
        data[filename] = df['Value']
    except Exception as e:
        print(f"Failed to read {file}: {e}")

# === LINE PLOT HELPER ===
def plot_line(data_dict, metrics, title, output_filename):
    fig, ax = plt.subplots(figsize=(10, 6))
    x = np.arange(len(metrics))

    for name, values in data_dict.items():
        y = [values.get(metric, 0) for metric in metrics]
        ax.plot(x, y, marker='o', label=name)

    ax.set_xticks(x)
    ax.set_xticklabels(metrics, rotation=30)
    ax.set_ylabel('Value')
    ax.set_title(title)
    ax.grid(True)
    ax.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, output_filename), dpi=300)
    plt.close()

# === RADAR PLOT HELPER ===
def plot_radar(data_dict, metrics, title, output_filename):
    labels = metrics
    num_vars = len(labels)
    angles = np.linspace(0, 2 * np.pi, num_vars, endpoint=False).tolist()
    angles += angles[:1]

    fig, ax = plt.subplots(figsize=(8, 8), subplot_kw=dict(polar=True))
    plt.title(title, size=16, y=1.08)

    for name, values in data_dict.items():
        values_list = [values.get(metric, 0) for metric in metrics]
        values_list += values_list[:1]
        ax.plot(angles, values_list, label=name)
        ax.fill(angles, values_list, alpha=0.1)

    ax.set_xticks(angles[:-1])
    ax.set_xticklabels(labels)
    ax.set_rlabel_position(250)
    ax.tick_params(colors='#888888')
    ax.grid(color='#CCCCCC')
    plt.legend(loc='upper right', bbox_to_anchor=(1.2, 1.1))
    plt.tight_layout()
    plt.savefig(os.path.join(output_folder, output_filename), dpi=300)
    plt.close()

# === CONSOLIDATE CSV DATA ===
consolidated_df = pd.DataFrame(data)
consolidated_df = consolidated_df.fillna(0)  # fill missing metrics with 0
consolidated_path = os.path.join(output_folder, "consolidated_metrics.csv")
consolidated_df.to_csv(consolidated_path)
print(f"📊 Consolidated CSV saved at: {consolidated_path}")

# === PLOT EACH GROUP ===
plot_line(data, voxel_metrics, "Voxel Count Comparison", "voxel_comparison.png")
plot_line(data, tnfpfn_metrics, "TP/FP/TN/FN Comparison", "confusion_comparison.png")
plot_radar(data, quality_metrics, "Precision/Recall/F1/IoU/Completeness Comparison", "quality_comparison.png")

print(f"✅ Plots saved in: {output_folder}")
