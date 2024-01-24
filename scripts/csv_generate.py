import rospy
import rosbag
import csv

def convert_bag_to_csv(bag_file, csv_file):
    with rosbag.Bag(bag_file, 'r') as bag:
        with open(csv_file, 'w') as csvfile:
            csvwriter = csv.writer(csvfile)

            # Write the header row to the CSV file
            csvwriter.writerow(['Timestamp', 'Topic', 'Message'])

            # Iterate through the messages in the bag file
            for topic, msg, t in bag.read_messages():
                # Write the message to the CSV file
                csvwriter.writerow([t.to_sec(), topic, str(msg)])

if __name__ == '__main__':
    rospy.init_node('bag_to_csv_node', anonymous=True)

    bag_file_path = '/home/lyn/2024-01-07-10-35-02.bag'  # Replace with your bag file path
    csv_file_path = '/home/lyn/2024-01-07-10-35-02.csv'  # Replace with the desired CSV file path

    convert_bag_to_csv(bag_file_path, csv_file_path)
