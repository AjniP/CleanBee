import serial
import json
import pandas as pd

serial = serial.Serial("/dev/ttyUSB0", 115200, timeout=1)


def send_command():

    serial.write(b"SEND\n")

    line = serial.readline()
    jline = json.loads(line)
    
    with open("data.json",'w',encoding='utf-8') as f:
        json.dump(jline, f)

    serial.close()

def json_to_csv():

    json_file_path = "data.json"

    # Load JSON data from the file
    with open(json_file_path, "r") as file:
        json_data = json.load(file)

    flat_data = json_data["readings"][0]

    # Convert to DataFrame
    df = pd.DataFrame(flat_data)

    # Save to CSV
    csv_file_path = "csv_data.csv"
    df.to_csv(csv_file_path, index=False)

    csv_file_path

def main():
    send_command()
    json_to_csv()
    

main()