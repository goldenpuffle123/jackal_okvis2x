import sys
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import Reader

def analyze_bag(bag_path):
    # Update these topic names if they differ in your bag
    IMAGE_TOPIC = '/femtobolt/color/image_raw'
    DEPTH_TOPIC = '/femtobolt/depth/image_raw'
    IMU_TOPIC = '/femtobolt/gyro_accel/sample'
    # IMAGE_TOPIC = '/camera/camera/color/image_raw'
    # DEPTH_TOPIC = '/camera/camera/aligned_depth_to_color/image_raw' # placeholder
    # IMU_TOPIC = '/camera/camera/imu'
    # IMAGE_TOPIC = '/zed_node/left/image_rect_color'
    # DEPTH_TOPIC = '/zed_node/depth/depth_registered/image_raw' # placeholder
    # IMU_TOPIC = '/zed_node/imu/data'
    
    image_receive_times = []
    image_delays = []
    
    depth_receive_times = []
    depth_delays = []
    
    imu_receive_times = []
    imu_delays = []

    from pathlib import Path
    print(f"Opening bag: {bag_path}")
    
    try:
        with AnyReader([Path(bag_path)]) as reader:
            for connection, timestamp, rawdata in reader.messages():
                if connection.topic == IMAGE_TOPIC:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    
                    # Convert to seconds
                    receive_time = timestamp * 1e-9
                    header_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
                    
                    image_receive_times.append(receive_time)
                    # Calculate delay: Receive time (Host PC) - Header time (Hardware)
                    image_delays.append(receive_time - header_time)
                    
                elif connection.topic == DEPTH_TOPIC:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    
                    receive_time = timestamp * 1e-9
                    header_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
                    
                    depth_receive_times.append(receive_time)
                    depth_delays.append(receive_time - header_time)
                    
                elif connection.topic == IMU_TOPIC:
                    msg = reader.deserialize(rawdata, connection.msgtype)
                    
                    receive_time = timestamp * 1e-9
                    header_time = msg.header.stamp.sec + (msg.header.stamp.nanosec * 1e-9)
                    
                    imu_receive_times.append(receive_time)
                    imu_delays.append(receive_time - header_time)
                    
    except Exception as e:
        print(f"Error reading bag: {e}")
        sys.exit(1)

    if not image_receive_times or not imu_receive_times:
        print("Error: Could not find messages for one or both topics. Check your topic names.")
        sys.exit(1)

    # Normalize X-axis to start at 0 seconds for readability
    start_times = [image_receive_times[0], imu_receive_times[0]]
    if depth_receive_times:
        start_times.append(depth_receive_times[0])
    start_time = min(start_times)
    
    image_x = [t - start_time for t in image_receive_times]
    imu_x = [t - start_time for t in imu_receive_times]
    if depth_receive_times:
        depth_x = [t - start_time for t in depth_receive_times]

    # Plotting
    plt.figure(figsize=(10, 6))
    plt.plot(image_x, image_delays, label='Image Stream', color='blue', alpha=0.7)
    if depth_receive_times:
        plt.plot(depth_x, depth_delays, label='Depth Stream', color='green', alpha=0.7)
    plt.plot(imu_x, imu_delays, label='IMU Stream', color='red', alpha=0.7)
    
    plt.title('Time Delay: Host PC Receive Time vs Hardware Header Time')
    plt.xlabel('Bag Recording Time (seconds)')
    plt.ylabel('Delay (Receive Time - Header Time) [seconds]')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    
    print("Generating plot...")
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_timestamps.py /path/to/your/rosbag_directory")
        sys.exit(1)
        
    bag_directory = sys.argv[1]
    analyze_bag(bag_directory)