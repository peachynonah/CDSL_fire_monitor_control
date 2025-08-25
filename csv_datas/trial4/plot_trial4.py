import os
import sys
import pandas as pd
import matplotlib.pyplot as plt

def plot_joint_data(file_number, start_time=None, end_time=None):
    base_path = "/home/kiro/CDSL/CDSL_fire_monitor_control/csv_datas/trial4"
    csv_path = os.path.join(base_path, f"cdsl_data_{file_number}.csv")

    try:
        df = pd.read_csv(csv_path)
        required_cols = {"joint_pos_desired", "joint_pos", "joint_vel_desired", 
                         "joint_vel_filtered", "target_torque", 
                         "propo_term_torque", "deriv_term_torque", "current_time"}
        if required_cols.issubset(df.columns):
            df = df[list(required_cols)]
        else:
            df = pd.read_csv(csv_path, header=None)
            df = df.iloc[:, :8].copy()
            df.columns = ["joint_pos_desired", "joint_pos", "joint_vel_desired", 
                          "joint_vel_filtered", "target_torque", 
                          "propo_term_torque", "deriv_term_torque", "current_time"]
    except Exception as e:
        raise RuntimeError(f"Cannot read CSV file: {e}")

    for col in df.columns:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna(subset=list(required_cols))

    if start_time is not None:
        df = df[df["current_time"] >= start_time]
    if end_time is not None:
        df = df[df["current_time"] <= end_time]

    if df.empty:
        raise RuntimeError("No data in the selected time range.")

    # ??? ?? 3?? subplot
    fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

    # Joint Velocity Desired vs Filtered
    axs[0].plot(df["current_time"].to_numpy(), df["joint_vel_desired"].to_numpy(), label="joint_vel_desired", color="tab:blue")
    axs[0].plot(df["current_time"].to_numpy(), df["joint_vel_filtered"].to_numpy(), label="joint_vel_filtered", color="tab:orange")
    axs[0].set_ylabel("joint velocity")
    axs[0].set_title("Joint Velocity Desired vs Filtered")
    axs[0].grid(True)
    axs[0].legend()

    # Joint Position vs Desired Joint Position
    axs[1].plot(df["current_time"].to_numpy(), df["joint_pos"].to_numpy(), label="joint_pos", color="tab:green")
    axs[1].plot(df["current_time"].to_numpy(), df["joint_pos_desired"].to_numpy(), label="joint_pos_desired", color="tab:red")
    axs[1].set_ylabel("joint position")
    axs[1].set_title("Joint Position vs Desired Joint Position (time)")
    axs[1].grid(True)
    axs[1].legend()

    # Torque components vs Current Time (??? ??)
    axs[2].plot(df["current_time"].to_numpy(), df["target_torque"].to_numpy(), label="target_torque", color="tab:purple", linewidth=2.5)
    axs[2].plot(df["current_time"].to_numpy(), df["propo_term_torque"].to_numpy(), label="propo_term_torque", color="tab:blue", linestyle=':', linewidth=4.5)
    axs[2].plot(df["current_time"].to_numpy(), df["deriv_term_torque"].to_numpy(), label="deriv_term_torque", color="tab:orange", linestyle='-.', linewidth=3.5)
    axs[2].set_xlabel("time")
    axs[2].set_ylabel("torque")
    axs[2].set_title("Torque Components vs Time")
    axs[2].grid(True)
    axs[2].legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) > 1:
        file_number = sys.argv[1]
        plot_joint_data(file_number)
    else:
        print("Usage: python3 plot.py <file_number>")
