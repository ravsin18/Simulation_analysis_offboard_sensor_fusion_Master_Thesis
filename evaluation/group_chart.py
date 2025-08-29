import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


def calculate_and_prepare_data(file_path_noise, file_path_no_noise):
    """
    Loads data, calculates metrics, and correctly reshapes the data for plotting.
    Handles flexible column header formats like:
      - prefix_SENSORID_suffix (e.g. warehouse_S1_noise)
      - prefix_SENSORID       (e.g. warehouse_S1)
      - prefix_ID             (e.g. warehouse_A, warehouse_B, etc.)
    """

    try:
        df_noise = pd.read_csv(file_path_noise, index_col='Metric')
        df_no_noise = pd.read_csv(file_path_no_noise, index_col='Metric')
    except FileNotFoundError as e:
        print(f"ERROR: Could not find a file. Please ensure both CSV files are in the same directory as the script.")
        print(f"Details: {e}")
        return None

    # --- Data Cleaning and Standardization ---
    def extract_sensor_id(col):
        parts = col.split('_')
        if len(parts) >= 2:
            # Take the last element (works for both 'warehouse_S1' and 'warehouse_S1_noise')
            return parts[-1].upper()
        else:
            # If column doesn't follow expected pattern, just uppercase it
            return col.upper()

    df_noise.columns = [extract_sensor_id(c) for c in df_noise.columns]
    df_no_noise.columns = [extract_sensor_id(c) for c in df_no_noise.columns]

    sensors = ['S1', 'A', 'B', 'C', 'S2']  # Desired order for plotting

    # --- Metric Calculation ---
    results = {}
    for condition, df in [('With Noise', df_noise), ('Without Noise', df_no_noise)]:
        try:
            tp = pd.to_numeric(df.loc['3D True Positives'], errors='coerce')
            fp = pd.to_numeric(df.loc['3D False Positives'], errors='coerce')
            fn = pd.to_numeric(df.loc['3D False Negatives'], errors='coerce')
            completeness = pd.to_numeric(df.loc['Completeness (%)'], errors='coerce') / 100
        except KeyError as e:
            print(f"ERROR: A required metric row is missing in '{condition}' data. Metric not found: {e}")
            return None

        precision = (tp / (tp + fp)).fillna(0)
        recall = (tp / (tp + fn)).fillna(0)
        f1_score = (2 * (precision * recall) / (precision + recall)).fillna(0)
        iou = (tp / (tp + fp + fn)).fillna(0)

        metrics_df = pd.DataFrame({
            'Completeness': completeness, 'Precision': precision, 'Recall': recall,
            'F1 Score': f1_score, 'IoU': iou
        }).T
        results[condition] = metrics_df

    # --- Combine and Reshape Data ---
    df_combined = pd.concat(results)
    df_plot = df_combined.T
    df_plot.columns = df_plot.columns.swaplevel(0, 1)
    df_plot = df_plot.reindex(sensors)
    df_plot.sort_index(axis=1, inplace=True)

    return df_plot


def create_grouped_bar_chart(df_plot):
    """Generates and saves a grouped bar chart from the prepared data."""
    metrics = df_plot.columns.levels[0].unique()
    sensors_order = df_plot.index
    n_metrics = len(metrics)
    n_sensors = len(sensors_order)

    fig, axes = plt.subplots(n_metrics, 1, figsize=(12, 18), sharex=True)
    if n_metrics == 1:
        axes = [axes]

    bar_width = 0.35
    index = np.arange(n_sensors)
    colors = {'Without Noise': "#a5b41f", 'With Noise': "#ff0e0e"}

    for i, metric in enumerate(metrics):
        ax = axes[i]
        metric_data = df_plot[metric]
        no_noise_values = metric_data['Without Noise']
        noise_values = metric_data['With Noise']

        ax.bar(index - bar_width/2, no_noise_values, bar_width,
               label='Without Noise', color=colors['Without Noise'])
        ax.bar(index + bar_width/2, noise_values, bar_width,
               label='With Noise', color=colors['With Noise'])

        ax.set_ylabel('Score', fontweight='bold')
        ax.set_title(metric, fontweight='bold', fontsize=14)
        ax.set_xticks(index)
        ax.set_xticklabels(sensors_order, rotation=0)
        ax.legend()
        ax.grid(axis='y', linestyle='--', alpha=0.7)
        ax.set_ylim(0, 1.05)

    plt.xlabel('Sensor Configuration', fontweight='bold', fontsize=12)
    fig.suptitle('Sensor Performance Comparison: With vs. Without Noise', fontsize=18, y=0.99)
    plt.tight_layout(rect=[0, 0, 1, 0.98])

    plt.savefig('sensor_metrics_comparison.png', dpi=300, bbox_inches='tight')
    print("\nChart successfully generated and saved as 'sensor_metrics_comparison.png'")


# --- Main execution block ---
if __name__ == '__main__':
    try:
        noise_file = 'consolidated_metrics_with_noise.csv'
        no_noise_file = 'consolidated_metrics_without_noise.csv'

        plot_data = calculate_and_prepare_data(noise_file, no_noise_file)

        if plot_data is not None:
            if plot_data.isnull().values.any():
                print("\nWARNING: Missing data detected after calculations. Check CSV contents.")
                print("Consolidated Data Table (with missing values shown as NaN):")
                print(plot_data.to_string())
            else:
                create_grouped_bar_chart(plot_data)
                print("\n--- Consolidated Metrics Data ---")
                print(plot_data.to_string())

    except Exception as e:
        print(f"\nAn unexpected critical error occurred: {e}")
        import traceback
        traceback.print_exc()
