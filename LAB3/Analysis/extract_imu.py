import sqlite3
from pathlib import Path

bag_path = Path("../Data/stationary_bag/rosbag2_2026_03_11-22_54_21_0.db3")

conn = sqlite3.connect(str(bag_path))
cursor = conn.cursor()

cursor.execute("SELECT COUNT(*) FROM messages")
count = cursor.fetchone()[0]
print("Total messages:", count)

cursor.execute("SELECT timestamp FROM messages LIMIT 5")
rows = cursor.fetchall()
print("First 5 timestamps:")
for row in rows:
    print(row[0])

conn.close()
