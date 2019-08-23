import glob
import os
import time

disqualified_racers = set()
finished_racers = set()

def open_file():
    list_of_files = glob.glob('*.log')
    latest_file = max(list_of_files, key=os.path.getctime)
    print("Opened file: " + latest_file)
    return open(latest_file, "r+")

def follow(filename):
    filename.seek(0,2)
    while True:
        line = filename.readline()
        if not line:
            time.sleep(0.1)
            continue
        yield line  

def process(line):
    tokens = line.split()
    if len(tokens) != 3:
        print("ERROR Bad line: " + line)
        print("Tokens: " + str(tokens))
        return

    if tokens[1] == "disqualified" and tokens[2] == '1' and tokens[0] not in disqualified_racers:
        disqualified_racers.add(tokens[0])
        handle_disqualified_racer(tokens[0])
        return

    if tokens[1] == "finished" and tokens[2] == '1' and tokens[0] not in finished_racers:
        finished_racers.add(tokens[0])
        handle_finished_racer(tokens[0])
        return

def handle_disqualified_racer(racer_name):
    print(racer_name + " has been disqualified!")
    #Start a new race.

def handle_finished_racer(racer_name):
    print(racer_name + " has finished!")
    #Start a new race.

def main():
    f = open_file()
    for line in follow(f):
        process(line)

if __name__ == "__main__":
    main() 