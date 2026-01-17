# ---
# jupyter:
#   jupytext:
#     text_representation:
#       extension: .py
#       format_name: percent
#       format_version: '1.3'
#       jupytext_version: 1.18.1
#   kernelspec:
#     display_name: .venv
#     language: python
#     name: python3
# ---

# %%
import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from pathlib import Path

from evo.tools import plot
from evo.tools import settings

try:
    from IPython.display import display
except ImportError:
    def display(*args):
        print(*args)

# %%
BAGS_DIR = Path("../../bags")
OUTPUT_DIR = BAGS_DIR / "batch"
OUTPUT_DIR.mkdir(exist_ok=True, parents=True)

print(f"Saving results to: {OUTPUT_DIR.resolve()}")

# %%
def load_all_metrics(base_dir):
    all_data = []

    print(f"Starting batch processing in: {base_dir.resolve()}")
    
    bag_dirs = [d for d in base_dir.iterdir() if d.is_dir() and d.name != "batch"]
    print(f"Found {len(bag_dirs)} directories.")

    for bag_dir in bag_dirs:
        print(f"Processing: {bag_dir.name}")

        for metrics_file in bag_dir.glob("metrics_*.csv"):
            try:
                agent_name = metrics_file.stem.split("_")[1]

                df = pd.read_csv(metrics_file)
                df["Bag"] = bag_dir.name
                df["Agent"] = agent_name
                print(f"    Loaded {len(df)} rows from {metrics_file.name}")
                all_data.append(df)
            except Exception as e:
                print(f"    Error loading {metrics_file.name}: {e}")

    if not all_data:
        print("No data loaded.")
        return pd.DataFrame()

    return pd.concat(all_data, ignore_index=True)

# %%
df_all = load_all_metrics(BAGS_DIR)

if df_all.empty:
    print("No metrics found!")
else:
    LABEL_MAPPING = {
        "global": "FGO",
        "global_tm": "TM",
        "global_ekf": "EKF",
        "global_ukf": "UKF",
        "global_iekf": "IEKF",
    }
    
    df_all["Algorithm"] = df_all["Topic"].apply(lambda x: x.split("/")[-1] if "/" in x else x)
    df_all["Algorithm"] = df_all["Algorithm"].map(LABEL_MAPPING).fillna(df_all["Algorithm"])
    
    df_all["Algorithm"] = df_all["Algorithm"].map(LABEL_MAPPING).fillna(df_all["Algorithm"])
    
    print(f"Total records loaded: {len(df_all)}")
    print(f"Algorithms found: {df_all['Algorithm'].unique()}")
    display(df_all.head())

# %%
settings.SETTINGS.plot_seaborn_style = "whitegrid"
settings.SETTINGS.plot_usetex = True
settings.SETTINGS.plot_fontfamily = "serif"
settings.SETTINGS.plot_linewidth = 1.0
settings.SETTINGS.plot_fontscale = 1.0
plot.apply_settings(settings.SETTINGS)

plt.rcParams.update({
    'font.size': 10,
    'axes.titlesize': 10,
    'axes.labelsize': 10,
    'xtick.labelsize': 8,
    'ytick.labelsize': 8,
    'legend.fontsize': 8,
    'figure.figsize': (3.5, 2.5),
})

# %%
metadata_cols = ["Topic", "Bag", "Agent", "Algorithm", "Unnamed: 0"]
metric_cols = [c for c in df_all.columns if c not in metadata_cols]

for metric in metric_cols:

    print(f"Generating plots for {metric}...")
    # --- Violin Plot ---
    fig_violin = plt.figure(figsize=(3.5, 3.0))
    ax_violin = plot.prepare_axis(fig_violin, plot.PlotMode.xy)
    
    sns.violinplot(data=df_all, x="Algorithm", y=metric, hue="Algorithm", palette="muted", inner="box")
    
    plt.ylabel(metric)
    plt.xlabel("Algorithm")
    plt.xticks(rotation=45)
    
    filename_violin = f"violinplot_{metric.replace(' ', '_')}.png"
    filepath_violin = OUTPUT_DIR / filename_violin
    plt.savefig(filepath_violin, bbox_inches="tight", dpi=300)
    print(f"Saved {filepath_violin}")
    display(fig_violin)
    plt.close(fig_violin)

    # --- Box Plot ---
    fig_box = plt.figure(figsize=(3.5, 3.0))
    
    sns.boxplot(data=df_all, x="Algorithm", y=metric, hue="Algorithm", palette="muted")
    
    plt.ylabel(metric)
    plt.xlabel("Algorithm")
    plt.xticks(rotation=45)

    filename_box = f"boxplot_{metric.replace(' ', '_')}.png"
    filepath_box = OUTPUT_DIR / filename_box
    plt.savefig(filepath_box, bbox_inches="tight", dpi=300)
    print(f"Saved {filepath_box}")
    display(fig_box)
    plt.close(fig_box)

# %%
stats_list = []
for metric in metric_cols:
    grouped = df_all.groupby("Algorithm")[metric]
    mean_val = grouped.mean()
    median_val = grouped.median()
    
    df_stats = pd.DataFrame({"Mean": mean_val, "Median": median_val})
    df_stats["Metric"] = metric
    stats_list.append(df_stats)

if stats_list:
    df_final_stats = pd.concat(stats_list)
    df_final_stats = df_final_stats.reset_index().set_index(["Metric", "Algorithm"]).sort_index()
    
    stats_csv_path = OUTPUT_DIR / "statistics.csv"
    df_final_stats.to_csv(stats_csv_path)
    print(f"Statistics summary saved to: {stats_csv_path}")
    display(df_final_stats)
