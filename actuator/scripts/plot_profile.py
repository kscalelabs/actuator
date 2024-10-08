import matplotlib.pyplot as plt
import argparse

CUTOFF = 10000

def read_log_file(filename):
    times = []
    desired_positions = []
    current_positions = []

    with open(filename, 'r') as file:
        for line in file:
            parts = line.strip().split(',')
            time_str = parts[0].split(':')[1].strip().replace('s', '')
            desired_pos_str = parts[1].split(':')[1].strip()
            current_pos_str = parts[2].split(':')[1].strip()

            # times.append(float(time_str))
            # Append the timestep index instead of the actual time
            times.append(len(times))  # Use the length of the list as the timestep index
            desired_positions.append(float(desired_pos_str))
            current_positions.append(float(current_pos_str))

    return times, desired_positions, current_positions

def plot_positions(times, desired_positions, current_positions, save_path=None):
    times = times[:CUTOFF]
    desired_positions = desired_positions[:CUTOFF]
    current_positions = current_positions[:CUTOFF]

    plt.figure(figsize=(10, 6))
    plt.plot(times, desired_positions, label='Desired Position', color='blue')
    plt.plot(times, current_positions, label='Current Position', color='red')
    plt.xlabel('Timestep')
    plt.ylabel('Position')
    plt.title('Desired vs Current Position Over Time')
    plt.legend()
    plt.grid(True)
    
    if save_path:
        plt.savefig(save_path)
    else:
        plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Plot positions from a log file.')
    parser.add_argument('filename', nargs='?', default='actuator/cpp/position_log.txt', help='The log file to read from (default: actuator/cpp/position_log.txt)')
    parser.add_argument('--save', default="plot.png", help='Path to save the plot image (default: plot.png)')
    args = parser.parse_args()

    times, desired_positions, current_positions = read_log_file(args.filename)
    plot_positions(times, desired_positions, current_positions, args.save)
