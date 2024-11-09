#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function, division  # For Python 2/3 compatibility
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import math
import numpy as np

# Function to convert time to relative time
def convert_time_to_relative(input_filename, output_filename):
    with open(input_filename, 'r') as infile, open(output_filename, 'w') as outfile:
        reader = csv.reader(infile)
        writer = csv.writer(outfile)

        header = next(reader)
        writer.writerow(header)

        first_time = None
        rows = []
        
        for row in reader:
            if first_time is None:
                first_time = float(row[0])
            relative_time = round(float(row[0]) - first_time, 1)
            row[0] = relative_time
            rows.append(row)

        writer.writerows(rows)

# Function to read path data (x, y coordinates)
def read_path_data(file_path):
    x, y, time = [], [], []
    with open(file_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            x.append(float(row['x']))
            y.append(float(row['y']))
            time.append(float(row['time']))
    return x, y, time

# Function to calculate path length
def calculate_path_length(x, y):
    path_length = 0.0
    for i in range(1, len(x)):
        deltaS = math.sqrt((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)
        path_length += deltaS
    return path_length

# Function to calculate trajectory smoothness
def calculate_trajectory_smoothness(x, y):
    angles = []
    for i in range(1, len(x)-1):
        dx1 = x[i] - x[i-1]
        dy1 = y[i] - y[i-1]
        dx2 = x[i+1] - x[i]
        dy2 = y[i+1] - y[i]
        angle1 = math.atan2(dy1, dx1)
        angle2 = math.atan2(dy2, dx2)
        angle_change = abs(angle2 - angle1)
        angles.append(angle_change)
    return np.mean(angles)

# Function to read velocity data
def read_velocity_data(file_path):
    time, linear_velocity, angular_velocity = [], [], []
    with open(file_path, 'r') as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            time.append(float(row['time']))
            linear_velocity.append(float(row['linear_velocity']))
            angular_velocity.append(float(row['angular_velocity']))
    return time, linear_velocity, angular_velocity

# Function to calculate velocity metrics
def calculate_velocity_metrics(velocity):
    smoothness = np.var(velocity)
    average_velocity = np.mean(velocity)
    return smoothness, average_velocity

# Function to calculate acceleration
def calculate_acceleration(time, velocity):
    acceleration = []
    for i in range(1, len(velocity)):
        delta_v = velocity[i] - velocity[i-1]
        delta_t = time[i] - time[i-1]
        if delta_t != 0:
            acceleration.append(delta_v / delta_t)
        else:
            acceleration.append(0)
    return [0] + acceleration

# Function to calculate acceleration smoothness
def calculate_acceleration_smoothness(acceleration):
    return np.var(acceleration)

# Function to plot paths
def plot_poses_profile(x1, y1, x2, y2, x3, y3, output_file, dpi=600):
    fig, path_plot = plt.subplots(figsize=(10, 8))
    path_plot.grid(False)
    path_plot.set_xlabel('x [m]')
    path_plot.set_ylabel('y [m]')

    path_plot.plot(x1, y1, label='TEB')
    path_plot.plot(x2, y2, label='Smooth TEB')
    path_plot.plot(x3, y3, label='FLC-TEB')

    path_plot.set_title('Path Comparison')
    path_plot.legend()
    plt.savefig(output_file, dpi=dpi)
    plt.show()

# Function to plot velocity comparisons
def plot_velocity_comparison(time_data, linear_velocity_data, angular_velocity_data, output_file_prefix, labels, dpi=600):
    plt.figure(figsize=(10, 8))
    for i, (linear_velocity, label) in enumerate(zip(linear_velocity_data, labels)):
        plt.plot(time_data[i], linear_velocity, label='{}'.format(label))
    plt.ylabel('Linear Velocity [m/s]')
    plt.xlabel('Time [s]')
    plt.title('Linear Velocity Profile Comparison')
    plt.legend()
    plt.savefig('{}linear_velocity_profile.png'.format(output_file_prefix), dpi=dpi)
    plt.show()

    plt.figure(figsize=(10, 8))
    for i, (angular_velocity, label) in enumerate(zip(angular_velocity_data, labels)):
        plt.plot(time_data[i], angular_velocity, label='{}'.format(label))
    plt.ylabel('Angular Velocity [rad/s]')
    plt.xlabel('Time [s]')
    plt.title('Angular Velocity Profile Comparison')
    plt.legend()
    plt.savefig('{}angular_velocity_profile.png'.format(output_file_prefix), dpi=dpi)
    plt.show()

# Function to plot path length and time
def plot_path_length_and_time(path_lengths, path_times, categories, output_file, dpi=600):
    # Define the positions for the bars
    bar_positions = np.arange(len(categories))

    # Create a new figure and axis
    fig, ax1 = plt.subplots()

    # Define colors for the bars
    bar_colors = ['steelblue', 'darkorange', 'green']

    # Plot bar chart for path lengths with different colors and reduced width
    bars = ax1.bar(bar_positions, path_lengths, color=bar_colors, label='Path Length', width=0.5)  # Reduced width
    ax1.set_xlabel('Paths')  # Set the x-axis label
    ax1.set_ylabel('Length (units)', color='k')  # Set the y-axis label for length with black color

    # Dynamically set y-axis limits for path lengths
    min_length = min(path_lengths) * 0.95
    max_length = max(path_lengths) * 1.05
    ax1.set_ylim(min_length, max_length)
    ax1.set_yticks(np.linspace(min_length, max_length, 10))  # Adjusted y-axis ticks
    ax1.tick_params(axis='y', labelcolor='k')  # Set the y-axis tick color to black

    # Manually set the x-axis labels to match the order in `categories`
    ax1.set_xticks(bar_positions)
    ax1.set_xticklabels(categories)

    # Create a second y-axis to plot the line chart for path times
    ax2 = ax1.twinx()
    line, = ax2.plot(bar_positions, path_times, color='r', marker='o', linestyle='-', label='Path Time')
    ax2.set_ylabel('Time (s)', color='k')  # Set the y-axis label for time with black color

    # Dynamically set y-axis limits for path times
    min_time = min(path_times) * 0.95
    max_time = max(path_times) * 1.05
    ax2.set_ylim(min_time, max_time)
    ax2.set_yticks(np.linspace(min_time, max_time, 10))  # Adjusted y-axis ticks
    ax2.tick_params(axis='y', labelcolor='k')  # Set the y-axis tick color to black

    # Add a legend
    fig.legend(loc="upper left", bbox_to_anchor=(0.1, 0.9))

    # Show the plot
    plt.title('Path Length and Time')  # Set the title of the plot
    plt.savefig(output_file, dpi=dpi)
    plt.show()  # Display the plot

if __name__ == '__main__':
    # Filenames for the input and output
    input_files = ['path0_data_1.csv', 'path1_data_1.csv', 'path2_data_1.csv']
    output_files = ['path0_data_0.csv', 'path1_data_0.csv', 'path2_data_0.csv']

    # Step 1: Convert time to relative for each file
    for input_file, output_file in zip(input_files, output_files):
        convert_time_to_relative(input_file, output_file)
        print("Time conversion completed for {}. New file is: {}".format(input_file, output_file))

    # Step 2: Read and process path data
    labels = ['TEB', 'Smooth TEB', 'FLC-TEB']
    path_data = [read_path_data(file) for file in output_files]

    path_lengths = []
    path_times = []

    for i, (x, y, time) in enumerate(path_data):
        path_length = calculate_path_length(x, y)
        trajectory_smoothness = calculate_trajectory_smoothness(x, y)
        path_lengths.append(path_length)
        path_times.append(time[-1] - time[0])  # Assuming the last time minus the first time gives the total path time
        print("\nTrajectory {} length: {:.2f} m".format(labels[i], path_length))
        print("Trajectory {} smoothness (average direction change): {:.4f}".format(labels[i], trajectory_smoothness))

    # Step 3: Plot path comparison
    plot_poses_profile(path_data[0][0], path_data[0][1], path_data[1][0], path_data[1][1], path_data[2][0], path_data[2][1], 'path_comparison.png')

    # Step 4: Read and process velocity data
    velocity_data = [read_velocity_data(file) for file in output_files]

    for i, (time, linear_velocity, angular_velocity) in enumerate(velocity_data):
        linear_velocity_smoothness, avg_linear_velocity = calculate_velocity_metrics(linear_velocity)
        angular_velocity_smoothness, avg_angular_velocity = calculate_velocity_metrics(angular_velocity)
        linear_acceleration = calculate_acceleration(time, linear_velocity)
        angular_acceleration = calculate_acceleration(time, angular_velocity)
        linear_acceleration_smoothness = calculate_acceleration_smoothness(linear_acceleration)
        angular_acceleration_smoothness = calculate_acceleration_smoothness(angular_acceleration)
        
        print("Trajectory {} linear velocity smoothness: {:.4f}, average linear velocity: {:.2f} m/s".format(labels[i], linear_velocity_smoothness, avg_linear_velocity))
        print("Trajectory {} angular velocity smoothness: {:.4f}, average angular velocity: {:.2f} rad/s".format(labels[i], angular_velocity_smoothness, avg_angular_velocity))
        print("Trajectory {} linear acceleration smoothness: {:.4f} m²/s⁴".format(labels[i], linear_acceleration_smoothness))
        print("Trajectory {} angular acceleration smoothness: {:.4f} rad²/s⁴".format(labels[i], angular_acceleration_smoothness))
    
    # Step 5: Plot velocity comparison
    plot_velocity_comparison([data[0] for data in velocity_data], [data[1] for data in velocity_data], [data[2] for data in velocity_data], 'velocity_comparison_', labels)

    # Step 6: Plot path length and time
    plot_path_length_and_time(path_lengths, path_times, labels, 'path_length_and_time.png')